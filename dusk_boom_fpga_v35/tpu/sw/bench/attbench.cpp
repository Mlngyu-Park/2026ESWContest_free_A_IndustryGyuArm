// attbench.cpp — time the board attention kernel alone (cache-resident vs 10 layers streaming)
#define main rachel_main
#include "../rachel_tpu.cpp"
#undef main
int main() {
    const int H = 8, hd = 56, d = 448, ctx = 768, L = 10, pos = 100;
    vector<KV> kv(L);
    for (auto& c : kv) {
        c.k.resize((size_t)ctx * d); c.v.resize((size_t)ctx * d); c.ctx = ctx; c.hd = hd; c.H = H;
        for (size_t i = 0; i < c.k.size(); i++) { c.k[i] = (kv_t)(urand() - 0.5f); c.v[i] = (kv_t)(urand() - 0.5f); }
    }
    vector<float> q(d), h(d), att(ctx);
    for (auto& x : q) x = urand() - 0.5f;
    for (int mode = 0; mode < 2; mode++) {
        const int reps = 200;
        double t0 = secs();
        for (int r = 0; r < reps; r++) {
            const KV& c = kv[mode ? r % L : 0];
            attend_heads_neon(q.data(), pos, c, h.data(), att.data(), d, hd, 0, H);
        }
        double dt = secs() - t0;
        printf("%s: %.1f us per layer call, %.0f cycles per (head,row)\n", mode ? "10 layers" : "1 layer  ",
               dt / reps * 1e6, dt / reps / (H * (pos + 1)) * 666e6);
    }
    // parts: scores only / exp only
    {
        const KV& c = kv[0]; const int reps = 200; double t0 = secs(); float acc = 0;
        for (int r = 0; r < reps; r++) for (int i = 0; i <= pos; i++) { att[i] = fast_exp(att[i] - 1.f); acc += att[i]; }
        printf("exp: %.0f cycles per row (%g)\n", (secs() - t0) / reps / (pos + 1) * 666e6, acc);
        (void)c;
    }
    return 0;
}
