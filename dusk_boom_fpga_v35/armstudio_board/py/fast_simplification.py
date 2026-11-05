"""fast_simplification stand-in for the DUSK_BOOM board (the real package is a C++ extension with no
32-bit ARM build).  Same call: simplify(points, triangles, target_reduction) -> (points, faces).
Method: vertex clustering - snap the vertices to a grid, merge each cell into its mean, drop the faces
that collapse.  The grid is the finest one that keeps at most (1 - target_reduction) of the faces."""
import numpy as np

__version__ = "0-dusk-numpy"


def _cluster(v, f, lo, cell, n):
    k = np.floor((v - lo) / cell).astype(np.int64)
    np.clip(k, 0, n, out=k)
    key = (k[:, 0] * (n + 1) + k[:, 1]) * (n + 1) + k[:, 2]
    uniq, inv = np.unique(key, return_inverse=True)
    nf = inv.reshape(-1)[f]
    keep = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
    return inv.reshape(-1), len(uniq), nf[keep]


def simplify(points, triangles, target_reduction=0.5, agg=7, verbose=False,
             return_collapses=False, **kw):
    v = np.asarray(points, dtype=np.float64)
    f = np.asarray(triangles, dtype=np.int64)
    if len(f) < 8 or target_reduction <= 0:
        return v.astype(np.float32), f
    want = max(4, int(len(f) * (1.0 - float(target_reduction))))
    lo = v.min(0)
    ext = float((v.max(0) - lo).max()) or 1.0
    best = None
    n_lo, n_hi = 2, 2048
    while n_lo <= n_hi:                        # largest grid with face count <= want
        n = (n_lo + n_hi) // 2
        inv, nv, nf = _cluster(v, f, lo, ext / n, n)
        if len(nf) > want:
            n_hi = n - 1
        else:
            best = (inv, nv, nf)
            n_lo = n + 1
    if best is None:
        best = _cluster(v, f, lo, ext / 2, 2)
    inv, nv, nf = best
    pts = np.zeros((nv, 3))
    np.add.at(pts, inv, v)
    pts /= np.maximum(np.bincount(inv, minlength=nv), 1)[:, None]
    nf = np.unique(np.sort(nf, axis=1), axis=0) if len(nf) else nf
    used = np.unique(nf)                       # drop unreferenced vertices
    remap = np.full(nv, -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    out_v, out_f = pts[used].astype(np.float32), remap[nf]
    if verbose:
        print(f"fast_simplification(dusk): {len(f)} -> {len(out_f)} faces")
    if return_collapses:
        return out_v, out_f, np.zeros((0, 2), dtype=np.int64)
    return out_v, out_f
