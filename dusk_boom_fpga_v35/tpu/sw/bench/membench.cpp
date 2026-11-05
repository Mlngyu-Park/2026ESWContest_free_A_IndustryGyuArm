// membench.cpp — read speed of the TPU result buffer (reserved, /dev/mem) with different load widths
#include <arm_neon.h>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
using namespace std::chrono;
static double now() { return duration<double>(steady_clock::now().time_since_epoch()).count(); }
int main(int argc, char** argv) {
    int sync = argc > 1 ? atoi(argv[1]) : 1;
    int fd = open("/dev/mem", O_RDWR | (sync ? O_SYNC : 0));
    volatile uint8_t* m = (volatile uint8_t*)mmap(nullptr, 0x2000000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x1C000000);
    if (m == MAP_FAILED) { perror("mmap"); return 1; }
    const uint32_t Y = 0x1E80000; const int N = 65536;   // 64K rows x 16 B = 1 MB
    double t; int32_t s = 0;
    t = now(); for (int i = 0; i < N; i++) s += ((volatile int32_t*)(m + Y))[4 * i];
    printf("4B load per row        : %.1f ns/row\n", (now() - t) * 1e9 / N);
    t = now(); int32x4_t a = vdupq_n_s32(0);
    for (int i = 0; i < N; i++) a = vaddq_s32(a, vld1q_s32((const int32_t*)(m + Y + 16 * i)));
    printf("16B vld1 per row       : %.1f ns/row\n", (now() - t) * 1e9 / N);
    t = now();
    for (int i = 0; i < N; i += 4) {
        const int32_t* p = (const int32_t*)(m + Y + 16 * i);
        asm volatile("vldmia %1, {d16-d23}\n\tvadd.i32 q8, q8, q9\n\tvadd.i32 q10, q10, q11\n\tvadd.i32 q8, q8, q10\n\tvadd.i32 %q0, %q0, q8"
                     : "+w"(a) : "r"(p) : "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23", "memory");
    }
    printf("64B vldm per 4 rows    : %.1f ns/row\n", (now() - t) * 1e9 / N);
    // write: 8B stores vs 64B vstm
    t = now(); for (int i = 0; i < N * 2; i++) ((volatile uint64_t*)(m + 0x1E00000))[i] = i;
    printf("8B stores              : %.1f ns/16B\n", (now() - t) * 1e9 / N);
    t = now();
    for (int i = 0; i < N; i += 4) {
        int32_t* p = (int32_t*)(m + 0x1E00000 + 16 * i);
        asm volatile("vstmia %0, {d16-d23}" :: "r"(p) : "memory");
    }
    printf("64B vstm               : %.1f ns/16B\n", (now() - t) * 1e9 / N);
    printf("(sum %d %d)\n", s, vgetq_lane_s32(a, 0));
    return 0;
}
