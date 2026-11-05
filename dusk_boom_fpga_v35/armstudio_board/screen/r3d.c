/* r3d.c — small software 3D renderer for the ARM STUDIO board screen (no GPU on the Zynq-7010)
 *
 *   Draws triangle meshes into a 32-bit XRGB pixel buffer with a float depth buffer
 *   (z-test, flat shading per face, both sides lit). Called from armstudio_screen.py by ctypes:
 *
 *     r3d_mesh(rgb, zbuf, w, h, pitch_px, V, nv, F, nf, M, cam, color)
 *       rgb   uint32 pixels (pitch_px per row)      zbuf  float, w*h, cleared to +inf by the caller
 *       V     float[nv*3] vertices (mesh units)       F     int32[nf*3] faces
 *       M     float[12] model->world, rows 0..2 of a 4x4 (scale already folded in)
 *       cam   float[16]: R (3x3 world->eye, rows), target (3), dist, focal, cx, cy, light (3, world)
 *       color 0x00RRGGBB
 *
 *   Same projection as the Python version: c = R (P - target); z = dist - c.z;
 *   x = cx + focal c.x / z; y = cy - focal c.y / z.
 *
 *   build: arm-none-linux-gnueabihf-gcc -O3 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -ffast-math
 *          -shared -fPIC -nostdlib -o libr3d.so r3d.c        (no libc needed: no glibc version ties)
 */
#include <stdint.h>

#define NEAR 0.03f

static float vbuf[3 * 65536];          /* projected x, y, z per vertex (meshes up to 65536 vertices) */

static inline float fminf3(float a, float b, float c) { float m = a < b ? a : b; return m < c ? m : c; }
static inline float fmaxf3(float a, float b, float c) { float m = a > b ? a : b; return m > c ? m : c; }
static inline float fabsf_(float a) { return a < 0 ? -a : a; }
static inline float rsqrt_(float x)
{
    union { float f; uint32_t i; } u = { x };
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - 0.5f * x * y * y);
    return y * (1.5f - 0.5f * x * y * y);
}

int r3d_mesh(uint32_t *rgb, float *zbuf, int w, int h, int pitch_px,
             const float *V, int nv, const int32_t *F, int nf,
             const float *M, const float *cam, uint32_t color)
{
    if (nv > 65536)
        return -1;
    const float *R = cam, *T = cam + 9;
    const float dist = cam[12], foc = cam[13], cx = cam[14], cy = cam[15];
    const float *L = cam + 16;
    /* world-space light is fixed, the per-face normal is in world space too */


    /* vertices: model -> world -> eye -> screen */
    for (int i = 0; i < nv; i++) {
        const float *p = V + 3 * i;
        float x = M[0] * p[0] + M[1] * p[1] + M[2] * p[2] + M[3] - T[0];
        float y = M[4] * p[0] + M[5] * p[1] + M[6] * p[2] + M[7] - T[1];
        float z = M[8] * p[0] + M[9] * p[1] + M[10] * p[2] + M[11] - T[2];
        float ex = R[0] * x + R[1] * y + R[2] * z;
        float ey = R[3] * x + R[4] * y + R[5] * z;
        float ez = dist - (R[6] * x + R[7] * y + R[8] * z);
        float *o = vbuf + 3 * i;
        if (ez < NEAR) { o[2] = -1.0f; continue; }
        float iz = 1.0f / ez;
        o[0] = cx + foc * ex * iz;
        o[1] = cy - foc * ey * iz;
        o[2] = ez;
    }
    const float cr = (float)((color >> 16) & 255), cg = (float)((color >> 8) & 255), cb = (float)(color & 255);
    int drawn = 0;
    for (int f = 0; f < nf; f++) {
        const int a = F[3 * f], b = F[3 * f + 1], c = F[3 * f + 2];
        const float *A = vbuf + 3 * a, *B = vbuf + 3 * b, *C = vbuf + 3 * c;
        if (A[2] < 0 || B[2] < 0 || C[2] < 0)
            continue;
        float minx = fminf3(A[0], B[0], C[0]), maxx = fmaxf3(A[0], B[0], C[0]);
        float miny = fminf3(A[1], B[1], C[1]), maxy = fmaxf3(A[1], B[1], C[1]);
        if (maxx < 0 || maxy < 0 || minx >= w || miny >= h)
            continue;
        float area = (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]);
        if (fabsf_(area) < 1e-6f)
            continue;
        /* flat shade from the world-space normal (both sides lit, like the viewer) */
        const float *pa = V + 3 * a, *pb = V + 3 * b, *pc = V + 3 * c;
        float ux = pb[0] - pa[0], uy = pb[1] - pa[1], uz = pb[2] - pa[2];
        float vx = pc[0] - pa[0], vy = pc[1] - pa[1], vz = pc[2] - pa[2];
        float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        float wx = M[0] * nx + M[1] * ny + M[2] * nz;
        float wy = M[4] * nx + M[5] * ny + M[6] * nz;
        float wz = M[8] * nx + M[9] * ny + M[10] * nz;
        float n2 = wx * wx + wy * wy + wz * wz;
        float d = n2 > 0 ? (wx * L[0] + wy * L[1] + wz * L[2]) * rsqrt_(n2) : 0;
        float s = 0.36f + 0.64f * fabsf_(d);
        uint32_t px = ((uint32_t)(cr * s) << 16) | ((uint32_t)(cg * s) << 8) | (uint32_t)(cb * s);
        int x0 = minx < 0 ? 0 : (int)minx, x1 = maxx > w - 1 ? w - 1 : (int)maxx;
        int y0 = miny < 0 ? 0 : (int)miny, y1 = maxy > h - 1 ? h - 1 : (int)maxy;
        float ia = 1.0f / area;
        for (int y = y0; y <= y1; y++) {
            float py = y + 0.5f;
            uint32_t *row = rgb + (long)y * pitch_px;
            float *zr = zbuf + (long)y * w;
            for (int x = x0; x <= x1; x++) {
                float pxf = x + 0.5f;
                float w0 = ((B[0] - pxf) * (C[1] - py) - (B[1] - py) * (C[0] - pxf)) * ia;
                float w1 = ((C[0] - pxf) * (A[1] - py) - (C[1] - py) * (A[0] - pxf)) * ia;
                float w2 = 1.0f - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0)
                    continue;
                float z = w0 * A[2] + w1 * B[2] + w2 * C[2];
                if (z < zr[x]) {
                    zr[x] = z;
                    row[x] = px;
                }
            }
        }
        drawn++;
    }
    return drawn;
}
