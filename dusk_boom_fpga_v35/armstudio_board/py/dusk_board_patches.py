"""Board-only patches applied before ARM STUDIO is imported (run_armstudio.py).

trimesh.collision.CollisionManager needs python-fcl, which has no 32-bit ARM build.  This stand-in
keeps the same calls (add_object / set_transform / remove_object / in_collision_other /
in_collision_internal / in_collision_single) and tests whether vertices of one mesh lie inside the
other (trimesh contains = ray test, rtree), after a bounding-box check.  Approximation: two meshes
that cross only edge-through-face without any vertex inside are not seen.  The link meshes are dense
enough after decimation for the arm-vs-fixture checks ARM STUDIO does."""
import numpy as np
import trimesh
import trimesh.collision as _tc


def _have_fcl():
    try:
        import fcl  # noqa: F401
        return True
    except Exception:
        return False


def _world(mesh, T):
    v = np.asarray(mesh.vertices, dtype=np.float64)
    return v @ T[:3, :3].T + T[:3, 3]


def _inside(mesh, T, pts):
    """any of pts (world) inside mesh placed at T?"""
    if len(pts) == 0:
        return False
    loc = (pts - T[:3, 3]) @ T[:3, :3]                  # world -> mesh frame
    lo, hi = mesh.bounds
    sel = loc[np.all((loc >= lo) & (loc <= hi), axis=1)]
    if len(sel) == 0:
        return False
    try:
        if mesh.is_watertight:
            return bool(np.any(mesh.contains(sel)))
        return bool(np.any(mesh.convex_hull.contains(sel)))
    except Exception:
        return True                                     # unsure -> report contact (safe side)


def _aabb(v):
    return v.min(0), v.max(0)


class ApproxCollisionManager:
    def __init__(self):
        self._o = {}

    def add_object(self, name, mesh, geom=None, transform=None):
        T = np.eye(4) if transform is None else np.asarray(transform, dtype=np.float64)
        self._o[name] = [mesh, T]
        return name

    def remove_object(self, name):
        self._o.pop(name, None)

    def set_transform(self, name, transform):
        self._o[name][1] = np.asarray(transform, dtype=np.float64)

    def _pairs(self, other, same=False):
        items = list(self._o.items())
        oitems = items if same else list(other._o.items())
        cache = {n: _world(m, T) for n, (m, T) in items}
        if not same:
            cache.update({("o", n): _world(m, T) for n, (m, T) in oitems})
        hits = set()
        for i, (a, (ma, Ta)) in enumerate(items):
            va = cache[a]
            la, ha = _aabb(va)
            for j, (b, (mb, Tb)) in enumerate(oitems):
                if same and j <= i:
                    continue
                vb = cache[b] if same else cache[("o", b)]
                lb, hb = _aabb(vb)
                if np.any(ha < lb) or np.any(hb < la):
                    continue
                if _inside(mb, Tb, va) or _inside(ma, Ta, vb):
                    hits.add((a, b))
        return hits

    def in_collision_other(self, other_manager, return_names=False, return_data=False):
        hits = self._pairs(other_manager)
        out = [bool(hits)]
        if return_names:
            out.append(hits)
        if return_data:
            out.append([])
        return out[0] if len(out) == 1 else tuple(out)

    def in_collision_internal(self, return_names=False, return_data=False):
        hits = self._pairs(self, same=True)
        out = [bool(hits)]
        if return_names:
            out.append(hits)
        if return_data:
            out.append([])
        return out[0] if len(out) == 1 else tuple(out)

    def in_collision_single(self, mesh, transform=None, return_names=False, return_data=False):
        tmp = ApproxCollisionManager()
        tmp.add_object("__single__", mesh, transform=transform)
        hits = {(a, b) for b, a in tmp._pairs(self)}
        names = {a for a, _ in hits}
        out = [bool(hits)]
        if return_names:
            out.append(names)
        if return_data:
            out.append([])
        return out[0] if len(out) == 1 else tuple(out)


def apply():
    if not _have_fcl():
        _tc.CollisionManager = ApproxCollisionManager
        trimesh.collision.CollisionManager = ApproxCollisionManager
        print("[dusk] python-fcl missing -> approximate collision checks (vertex inside mesh)")
