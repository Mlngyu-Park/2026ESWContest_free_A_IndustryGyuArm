// tpu_hw.h — DUSK_BOOM TPU driver for rachel_tpu.cpp (Linux on the Zynq, /dev/mem, no kernel driver)
//
//   FPGA side: rtl/tpu_top.v (registers at 0x43C00000) + rtl/tpu_mv.v (2 engines on S_AXI_HP1/HP2)
//   Memory   : 32 MB at 0x1C000000, reserved (no-map) in the device tree — Linux never uses it
//                0x0000000 ..           weights, one block per tensor (4 KB aligned, row-major int8)
//                0x1E00000  XBUF        activations, int16 [m][k]
//                0x1E80000  YBUF0       engine 0 results, 16 B per row (int32 x 4 vectors)
//                0x1F00000  YBUF1       engine 1 results
//   One y = W x: rows are split between the 2 engines (engine 0 rows [0, r0), r0*K multiple of 128).
//   Device memory (no cache): only 8-byte aligned stores / 4-byte loads are used.
#include <fcntl.h>
#include <sys/file.h>
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif
#include <sys/mman.h>
#include <unistd.h>

namespace tpu {

static const uint32_t REG_PHYS = 0x43C00000u, MEM_PHYS = 0x1C000000u, MEM_SIZE = 0x02000000u;
static const uint32_t XBUF = 0x1E00000u, YBUF0 = 0x1E80000u, YBUF1 = 0x1F00000u;
static const uint32_t TAG = XBUF - 4096u;               // 8 bytes: which process owns the weights now
//   Sharing: ARM STUDIO may run several rachel processes (Double Rachel + its sampling twin, v9).
//   begin()/end() hold an flock for one whole answer; if another process uploaded its weights
//   since our last answer (TAG differs), our weight table is dropped and filled again on use.

struct Dev {
    volatile uint32_t* r = nullptr;
    volatile uint8_t* mem = nullptr;
    bool tried = false, ok = false;
    uint32_t next = 0;                                   // weight allocation pointer
    std::map<const int8_t*, uint32_t> woff;              // tensor data -> offset in TPU memory
    double t_hw = 0;                                     // seconds spent waiting for the TPU
    double t_q = 0, t_put = 0, t_reg = 0, t_rd = 0;      // quantize / copy x / registers / read y
    long calls = 0;
    std::vector<int16_t> xi;

    int lock_fd = -1;
    uint64_t my_tag = 0;
    void begin() {
        if (!init()) return;
        if (!my_tag) my_tag = ((uint64_t)getpid() << 32) ^ (uint64_t)chrono::steady_clock::now().time_since_epoch().count() | 1;
        if (lock_fd < 0) lock_fd = open("/tmp/dusk_tpu.lock", O_CREAT | O_RDWR, 0666);
        if (lock_fd >= 0) flock(lock_fd, LOCK_EX);
        volatile uint64_t* tp = (volatile uint64_t*)(mem + TAG);
        if (*tp != my_tag) { woff.clear(); next = 0; *tp = my_tag; }
    }
    void end() { if (lock_fd >= 0) flock(lock_fd, LOCK_UN); }

    uint32_t rd(uint32_t a) { return r[a / 4]; }
    void wr(uint32_t a, uint32_t v) { r[a / 4] = v; }

    bool init() {
        if (tried) return ok;
        tried = true;
        int fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (fd < 0) { perror("[tpu] /dev/mem"); return false; }
        void* pr = mmap(nullptr, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, REG_PHYS);
        void* pm = mmap(nullptr, MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MEM_PHYS);
        close(fd);
        if (pr == MAP_FAILED || pm == MAP_FAILED) { perror("[tpu] mmap"); return false; }
        r = (volatile uint32_t*)pr;
        mem = (volatile uint8_t*)pm;
        uint32_t id = rd(0x000);
        if (id != 0x44545055u) {
            fprintf(stderr, "[tpu] no TPU in the FPGA (ID %08x) - CPU is used\n", id);
            return false;
        }
        fprintf(stderr, "[tpu] TPU found: config %08x\n", rd(0x004));
        ok = true;
        return true;
    }

    // aligned copy into device memory (8-byte stores)
    void put(uint32_t off, const void* src, size_t n) {
        volatile uint64_t* d = (volatile uint64_t*)(mem + off);
        const uint8_t* s = (const uint8_t*)src;
        size_t i = 0;
        for (; i + 8 <= n; i += 8) { uint64_t v; memcpy(&v, s + i, 8); d[i / 8] = v; }
        if (i < n) { uint64_t v = 0; memcpy(&v, s + i, n - i); d[i / 8] = v; }
    }

    // weight block of a tensor (uploaded on first use)
    bool weights(const Tensor& W, int rows, int cols, uint32_t& off) {
        auto it = woff.find(W.q.data());
        if (it != woff.end()) { off = it->second; return true; }
        size_t n = (size_t)rows * cols;
        if (next + n > TAG) { fprintf(stderr, "[tpu] weight memory full\n"); return false; }
        off = next;
        put(off, W.q.data(), n);
        next = (uint32_t)((next + n + 4095) & ~4095u);
        woff[W.q.data()] = off;
        return true;
    }

    // 64-byte block moves (one NEON instruction each): device memory is uncached, so wide
    // accesses matter — measured on the board: 8 B stores 29 ns / 16 B, 64 B vstm 7 ns / 16 B
    static inline void cp64(volatile void* dst, const void* src) {
        asm volatile("vldmia %1, {d16-d23}\n\tvstmia %0, {d16-d23}"
                     :: "r"(dst), "r"(src) : "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "memory");
    }
    static inline void ld64(int32_t* dst, const volatile void* src) {
        asm volatile("vldmia %1, {d16-d23}\n\tvstmia %0, {d16-d23}"
                     :: "r"(dst), "r"(src) : "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "memory");
    }
    // copy n bytes (multiple of 16, src 8-byte aligned) into device memory
    void put16(uint32_t off, const void* src, size_t n) {
        const uint8_t* s = (const uint8_t*)src;
        size_t i = 0;
        for (; i + 64 <= n; i += 64) cp64(mem + off + i, s + i);
        if (i < n) put(off + (uint32_t)i, s + i, n - i);
    }

    // results of rows [0, nrows) of one engine: 16 B per row, int32 for vectors 0..3.
    // First a plain 64-byte copy out of the uncached buffer, then the scaling in normal memory.
    std::vector<int32_t> raw;
    std::map<const int8_t*, std::vector<float>> bias_f;  // bias as float, made once per tensor
    const float* biasf(const Tensor* B, int rows) {
        if (!B || !B->n) return nullptr;
        auto& v = bias_f[B->q.data()];
        if ((int)v.size() != rows) { v.resize(rows); for (int i = 0; i < rows; i++) v[i] = B->q[i] * B->scale; }
        return v.data();
    }
    void collect(uint32_t ybuf, int nrows, int row0, int n, const float* k, const float* bf, float* const* ys) {
        raw.resize((size_t)nrows * 4 + 16);
        for (int r = 0; r < nrows; r += 4) ld64(raw.data() + 4 * r, mem + ybuf + 16u * (uint32_t)r);
        const int32_t* q = raw.data();
#if defined(__ARM_NEON)
        if (n == 1) {                                     // 글자 생성 (벡터 하나): 4 줄씩 NEON 으로
            float* y = ys[0] + row0;
            const float32x4_t kv = vdupq_n_f32(k[0]);
            int i = 0;
            for (; i + 4 <= nrows; i += 4) {
                const int32x4x4_t r = vld4q_s32(q + 4 * i);
                float32x4_t v = vmulq_f32(vcvtq_f32_s32(r.val[0]), kv);
                if (bf) v = vaddq_f32(v, vld1q_f32(bf + row0 + i));
                vst1q_f32(y + i, v);
            }
            for (; i < nrows; i++) y[i] = q[4 * i] * k[0] + (bf ? bf[row0 + i] : 0.f);
            return;
        }
#endif
        for (int t = 0; t < n; t++) {
            float* y = ys[t] + row0;
            const float kt = k[t];
            if (bf) { const float* b = bf + row0; for (int i = 0; i < nrows; i++) y[i] = q[4 * i + t] * kt + b[i]; }
            else    { for (int i = 0; i < nrows; i++) y[i] = q[4 * i + t] * kt + 0.f; }
        }
    }

    // y_t = W x_t (+ B) for t < n (n <= 4): the weights are read once for all vectors
    bool mvn(const Tensor& W, const Tensor* B, const float* const* xs, float* const* ys, int n, int rows, int cols) {
        if (n < 1 || n > 4 || !init() || (cols & 7) || cols > 2048 || rows > 65535 || rows < 8) return false;
        uint32_t woff_;
        if (!weights(W, rows, cols, woff_)) return false;
        auto ta = chrono::steady_clock::now();
        xi.resize((size_t)n * cols);
        float sx[4];
        for (int t = 0; t < n; t++) sx[t] = quant16(xs[t], cols, xi.data() + (size_t)t * cols);
        auto tb = chrono::steady_clock::now();
        put16(XBUF, xi.data(), (size_t)n * cols * 2);
        auto tc = chrono::steady_clock::now();
        t_q += chrono::duration<double>(tb - ta).count();
        t_put += chrono::duration<double>(tc - tb).count();
        int r0 = rows / 2;
        while (r0 > 0 && ((long)r0 * cols) % 128) r0--;
        const uint32_t k8 = cols / 8;
        const uint32_t E0 = 0x100, E1 = 0x140;
        wr(E0 + 0x08, MEM_PHYS + woff_);                  wr(E1 + 0x08, MEM_PHYS + woff_ + (uint32_t)r0 * cols);
        wr(E0 + 0x0C, MEM_PHYS + XBUF);                   wr(E1 + 0x0C, MEM_PHYS + XBUF);
        wr(E0 + 0x10, MEM_PHYS + YBUF0);                  wr(E1 + 0x10, MEM_PHYS + YBUF1);
        wr(E0 + 0x14, k8);                                wr(E1 + 0x14, k8);
        wr(E0 + 0x18, (uint32_t)r0);                      wr(E1 + 0x18, (uint32_t)(rows - r0));
        wr(E0 + 0x1C, (uint32_t)n);                       wr(E1 + 0x1C, (uint32_t)n);
        __sync_synchronize();
        auto t0 = chrono::steady_clock::now();
        t_reg += chrono::duration<double>(t0 - tc).count();
        wr(0x00C, r0 > 0 ? 3u : 2u);                      // start both engines
        long spins = 0;
        uint32_t st;
        while ((st = rd(0x008)) & 3u) {
            if (++spins > 20000000) {
                fprintf(stderr, "[tpu] timeout (status %08x) - CPU from now on\n", st);
                ok = false;
                return false;
            }
        }
        __sync_synchronize();
        const double job_s = chrono::duration<double>(chrono::steady_clock::now() - t0).count();
        t_hw += job_s;
        if (trace_on()) {
            TR_calls++; TR_us += job_s * 1e6; TR_bytes += (double)rows * cols; TR_macs += (double)rows * cols * n;
            const double now_ms = chrono::duration<double, milli>(chrono::steady_clock::now().time_since_epoch()).count();
            if (now_ms - TR_last >= 20.0) {
                TR_last = now_ms;
                fprintf(stderr, "#C %d %s %d %d %d %ld %.0f %.0f %.0f\n", TR_L, TR_K, rows, cols, n, TR_calls, TR_us,
                        TR_bytes, TR_macs);
                fflush(stderr);
            }
        }
        calls++;
        auto t1 = chrono::steady_clock::now();
        float k[4];
        for (int t = 0; t < n; t++) k[t] = W.scale * sx[t];
        const float* bf = biasf(B, rows);
        collect(YBUF0, r0, 0, n, k, bf, ys);
        collect(YBUF1, rows - r0, r0, n, k, bf, ys);
        t_rd += chrono::duration<double>(chrono::steady_clock::now() - t1).count();
        return true;
    }
    bool mv(const Tensor& W, const Tensor* B, const float* x, float* y, int rows, int cols) {
        return mvn(W, B, &x, &y, 1, rows, cols);
    }
};

static Dev dev;

}  // namespace tpu

static bool tpu_hw_mv(const Tensor& W, const Tensor* B, const float* x, float* y, int rows, int cols) {
    return tpu::dev.mv(W, B, x, y, rows, cols);
}
static bool tpu_hw_mvn(const Tensor& W, const Tensor* B, const float* const* xs, float* const* ys, int n, int rows, int cols) {
    return tpu::dev.mvn(W, B, xs, ys, n, rows, cols);
}
