"""Thin helper layer over the SolidWorks COM API (SolidWorks 2026, pywin32 early binding).

All public helpers take millimetres and degrees; conversion to SolidWorks' native
metres/radians happens here. World frame: Y up, bucket axis = Y, Y=0 at bucket rim.
"""
import math
import os

import pythoncom
import win32com

# Keep the generated COM bindings next to the scripts: %TEMP%\gen_py gets wiped by Windows cleanup.
_GEN = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".gen_py")
os.makedirs(_GEN, exist_ok=True)
win32com.__gen_path__ = _GEN
import win32com.gen_py  # noqa: E402
win32com.gen_py.__path__ = [_GEN]

import win32com.client  # noqa: E402
from win32com.client import VARIANT, gencache  # noqa: E402

SW_TLB = r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\sldworks.tlb"
MM = 0.001
_mod = gencache.GetModuleForProgID("SldWorks.Application")
if _mod is None:
    from win32com.client import makepy
    makepy.GenerateFromTypeLibSpec(SW_TLB, bForDemand=False)
    gencache.Rebuild()
    _mod = gencache.GetModuleForProgID("SldWorks.Application")
if _mod is None:
    raise RuntimeError(f"Could not generate SolidWorks COM bindings from {SW_TLB}")


def cast(obj, iface):
    if obj is None:
        return None
    raw = obj._oleobj_ if hasattr(obj, "_oleobj_") else obj
    return getattr(_mod, iface)(raw)


def app():
    raw = win32com.client.dynamic.Dispatch("SldWorks.Application")
    sw = cast(raw, "ISldWorks")
    sw.Visible = True
    # Sketch automation hygiene: no dimension prompts, no inferencing snaps.
    sw.SetUserPreferenceToggle(10, False)   # swInputDimValOnCreate
    sw.SetUserPreferenceToggle(77, False)   # swSketchInference
    return sw


SW = None


def sw():
    global SW
    if SW is None:
        SW = app()
    return SW


def template(kind):
    idx = {"part": 8, "asm": 9, "drw": 10}[kind]
    return sw().GetUserPreferenceStringValue(idx)


class Part:
    """A part being built. Sketch coordinates are in mm on the chosen plane."""

    def __init__(self, name, folder):
        self.name = name
        self.folder = folder
        raw = sw().NewDocument(template("part"), 0, 0, 0)
        self.m = cast(raw, "IModelDoc2")
        self.ext = self.m.Extension
        self.sm = self.m.SketchManager
        self.fm = self.m.FeatureManager
        self.sm.AddToDB = True
        self.sm.DisplayWhenAdded = False
        self._planes = {}
        self._feat_n = 0

    # ---------- selection ----------
    def select(self, name, typ, append=False, mark=0):
        ok = self.ext.SelectByID2(name, typ, 0, 0, 0, append, mark, None, 0)
        if not ok:
            raise RuntimeError(f"select failed: {typ} {name}")

    def clear(self):
        self.m.ClearSelection2(True)

    # ---------- planes ----------
    def plane(self, base, offset_mm):
        """Offset reference plane parallel to base ('Top'/'Front'/'Right'). Positive = along base normal."""
        key = (base, round(offset_mm, 4))
        if offset_mm == 0:
            return f"{base} Plane"
        if key in self._planes:
            return self._planes[key]
        self.clear()
        self.select(f"{base} Plane", "PLANE")
        flags = 8 | (256 if offset_mm < 0 else 0)  # distance | flip
        rp = self.fm.InsertRefPlane(flags, abs(offset_mm) * MM, 0, 0, 0, 0)
        if rp is None:
            raise RuntimeError("InsertRefPlane failed")
        f = cast(self.m.FeatureByPositionReverse(0), "IFeature")
        nm = f"P_{base}_{offset_mm:g}".replace(".", "p").replace("-", "m")
        f.Name = nm
        self._planes[key] = nm
        self.clear()
        return nm

    # ---------- sketching ----------
    def sketch(self, plane_name):
        self.clear()
        self.select(plane_name, "PLANE")
        self.sm.InsertSketch(True)
        return self

    def end_sketch(self):
        self.sm.InsertSketch(True)
        nm = cast(self.m.FeatureByPositionReverse(0), "IFeature").Name
        self.clear()
        self.select(nm, "SKETCH")
        return nm

    def line(self, x1, y1, x2, y2):
        self.sm.CreateLine(x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0)

    def cline(self, x1, y1, x2, y2):
        self.sm.CreateCenterLine(x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0)

    def poly(self, pts, close=True):
        n = len(pts)
        for i in range(n if close else n - 1):
            (a, b), (c, d) = pts[i], pts[(i + 1) % n]
            self.line(a, b, c, d)

    def circle(self, x, y, r):
        self.sm.CreateCircleByRadius(x * MM, y * MM, 0, r * MM)

    def rect(self, x1, y1, x2, y2):
        self.poly([(x1, y1), (x2, y1), (x2, y2), (x1, y2)])

    def crect(self, cx, cy, w, h, angle_deg=0.0):
        a = math.radians(angle_deg)
        ca, sa = math.cos(a), math.sin(a)
        pts = []
        for dx, dy in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)):
            pts.append((cx + dx * ca - dy * sa, cy + dx * sa + dy * ca))
        self.poly(pts)

    def slot(self, x1, y1, x2, y2, r):
        """Stadium (obround) slot between two centres."""
        ang = math.atan2(y2 - y1, x2 - x1)
        nx, ny = -math.sin(ang) * r, math.cos(ang) * r
        self.line(x1 + nx, y1 + ny, x2 + nx, y2 + ny)
        self.line(x1 - nx, y1 - ny, x2 - nx, y2 - ny)
        # arcs: SketchManager.CreateArc(xc,yc,zc,x1,y1,z1,x2,y2,z2,dir)
        self.sm.CreateArc(x2 * MM, y2 * MM, 0, (x2 + nx) * MM, (y2 + ny) * MM, 0,
                          (x2 - nx) * MM, (y2 - ny) * MM, 0, -1)
        self.sm.CreateArc(x1 * MM, y1 * MM, 0, (x1 - nx) * MM, (y1 - ny) * MM, 0,
                          (x1 + nx) * MM, (y1 + ny) * MM, 0, -1)

    def arc_poly(self, cx, cy, r, a0, a1, n=24):
        return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
                 cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]

    def arc(self, cx, cy, r, a0_deg, a1_deg):
        """True sketch arc, counter-clockwise from a0 to a1."""
        a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
        self.sm.CreateArc(cx * MM, cy * MM, 0,
                          (cx + r * math.cos(a0)) * MM, (cy + r * math.sin(a0)) * MM, 0,
                          (cx + r * math.cos(a1)) * MM, (cy + r * math.sin(a1)) * MM, 0, 1)

    def annulus_sector(self, r0, r1, a0, a1):
        """Closed region between radii r0<r1 and angles a0<a1 (deg), true arcs."""
        def p(r, a):
            return (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
        self.arc(0, 0, r1, a0, a1)
        self.arc(0, 0, r0, a0, a1)
        self.line(*p(r0, a0), *p(r1, a0))
        self.line(*p(r0, a1), *p(r1, a1))

    # ---------- features ----------
    def _named(self, feat, name):
        f = cast(feat, "IFeature")
        if f is None:
            raise RuntimeError(f"feature failed: {name}")
        if name:
            try:
                f.Name = name
            except Exception:
                pass
        self.clear()
        return f

    def extrude(self, depth, name=None, mid=False, reverse=False, merge=True, depth2=None):
        self.end_sketch()
        if depth2 is not None:
            f = self.fm.FeatureExtrusion3(False, False, reverse, 0, 0, depth * MM, depth2 * MM,
                                          False, False, False, False, 0, 0, False, False,
                                          False, False, merge, True, True, 0, 0, False)
        else:
            f = self.fm.FeatureExtrusion3(True, False, reverse, 6 if mid else 0, 0,
                                          depth * MM, 0, False, False, False, False, 0, 0,
                                          False, False, False, False, merge, True, True, 0, 0, False)
        return self._named(f, name)

    def cut(self, depth, name=None, mid=False, reverse=False, through=False, both=False):
        """Cut-extrude along +sketch-normal (like extrude). SolidWorks' own default is -normal."""
        self.end_sketch()
        reverse = not reverse
        t1 = 1 if through else (6 if mid else 0)
        if both:
            f = self.fm.FeatureCut4(False, False, reverse, 1, 1, depth * MM, depth * MM,
                                    False, False, False, False, 0, 0, False, False, False, False,
                                    False, True, True, True, True, False, 0, 0, False, False)
        else:
            f = self.fm.FeatureCut4(True, False, reverse, t1, 0, depth * MM, 0,
                                    False, False, False, False, 0, 0, False, False, False, False,
                                    False, True, True, True, True, False, 0, 0, False, False)
        return self._named(f, name)

    def revolve(self, angle=360, name=None, cut=False, reverse=False, mid=False):
        self.end_sketch()
        t = 6 if mid else 0
        f = self.fm.FeatureRevolve2(True, True, False, cut, reverse, False, t, 0,
                                    math.radians(angle), 0, False, False, 0, 0, 0, 0, 0,
                                    True, True, True)
        return self._named(f, name)

    # ---------- appearance / material / output ----------
    def color(self, r, g, b, shine=0.3, transp=0.0):
        vals = VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [r, g, b, 1.0, 1.0, 0.35, shine, transp, 0.0])
        self.ext.SetMaterialPropertyValues(vals, 2, None)   # part-level, all configurations
        self.m.MaterialPropertyValues = vals
        self._coloured = True
        self._rgb = (r, g, b, transp)

    def material(self, name, db="SOLIDWORKS Materials"):
        part = cast(self.m, "IPartDoc")
        part.SetMaterialPropertyName2("", db, name)

    def mass_g(self):
        mp = self.ext.CreateMassProperty()
        mp = cast(mp, "IMassProperty")
        return mp.Mass * 1000.0

    def bbox(self):
        part = cast(self.m, "IPartDoc")
        b = part.GetPartBox(True)
        return [round(v / MM, 2) for v in b]

    def save(self, stl=True, step=False):
        os.makedirs(self.folder, exist_ok=True)
        path = os.path.join(self.folder, self.name + ".SLDPRT")
        self.m.ForceRebuild3(False)
        err = self.m.SaveAs3(path, 0, 1)
        if err != 0:
            raise RuntimeError(f"save failed ({err}) {path}")
        if stl:
            stl_dir = os.path.join(os.path.dirname(self.folder), "stl")
            os.makedirs(stl_dir, exist_ok=True)
            self.m.SaveAs3(os.path.join(stl_dir, self.name + ".STL"), 0, 3)
        if step:
            step_dir = os.path.join(os.path.dirname(self.folder), "step")
            os.makedirs(step_dir, exist_ok=True)
            self.m.SaveAs3(os.path.join(step_dir, self.name + ".STEP"), 0, 3)
        return path

    def snapshot(self, png_path, view="zup"):
        hide_refs(self.m)
        if view == "zup":
            zup_view(self.m)
        else:
            self.m.ShowNamedView2(view, -1)
        self.m.ViewZoomtofit2()
        self.m.GraphicsRedraw2()
        self.m.SaveAs3(png_path, 0, 3)

    def close(self):
        sw().CloseDoc(self.m.GetTitle())


def zup_view(m, eye=(1.0, -1.25, 0.9)):
    """Orient the active view for a Z-up model: camera along `eye` (model coords), Z vertical on screen."""
    import numpy as np
    back = np.array(eye, float); back /= np.linalg.norm(back)
    right = np.cross([0, 0, 1.0], back); right /= np.linalg.norm(right)
    up = np.cross(back, right)
    R = np.vstack([right, up, back])          # rows: view axes in model coords
    data = list(R.T.flatten()) + [0, 0, 0, 1.0, 0, 0, 0]
    mu = cast(sw().GetMathUtility(), "IMathUtility")
    t = mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, data))
    v = cast(m.ActiveView, "IModelView")
    v.Orientation3 = t
    m.ViewZoomtofit2()
    m.GraphicsRedraw2()


def hide_refs(m):
    """Hide planes/axes/origins/sketches and draw shaded without edges (tessellated arm meshes)."""
    try:
        m.Extension.SetUserPreferenceToggle(198, 0, True)
        m.ViewDisplayShaded()
        m.GraphicsRedraw2()
    except Exception:
        pass


def xform(rot_y=0.0, rot_x=0.0, rot_z=0.0, t=(0, 0, 0)):
    """Rigid transform for SolidWorks components: applied as rotations X, then Z, then Y, then translation (mm).

    SolidWorks MathTransform ArrayData uses row-vector convention: p' = p * R + T.
    """
    def rx(a):
        c, s = math.cos(a), math.sin(a)
        return [[1, 0, 0], [0, c, -s], [0, s, c]]

    def ry(a):
        c, s = math.cos(a), math.sin(a)
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]

    def rz(a):
        c, s = math.cos(a), math.sin(a)
        return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

    def mul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    # column-vector rotation M = Ry * Rz * Rx
    M = mul(ry(math.radians(rot_y)), mul(rz(math.radians(rot_z)), rx(math.radians(rot_x))))
    # row-vector convention needs transpose
    R = [M[j][i] for i in range(3) for j in range(3)]
    data = R + [t[0] * MM, t[1] * MM, t[2] * MM, 1.0, 0.0, 0.0, 0.0]
    mu = cast(sw().GetMathUtility(), "IMathUtility")
    return mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, data))


# ---------- text & face colours (appended helpers) ----------
def _text(self, cx, y_base, s, height, bold=True, mirror=False):
    """Centred sketch text on the active sketch (mm). Width estimated for SolidWorks' default font."""
    w = len(s) * 0.742 * height
    x0 = cx + (w / 2 if mirror else -w / 2)
    st = cast(self.m.InsertSketchText(x0 * MM, y_base * MM, 0, s, 0, 0, 1 if mirror else 0, 100, 100), "ISketchText")
    tf = cast(st.GetTextFormat(), "ITextFormat")
    tf.CharHeight = height * MM
    tf.Bold = bold
    st.SetTextFormat(False, tf)


def _color_faces(self, feat, r, g, b, shine=0.4):
    vals = VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [r, g, b, 1.0, 1.0, 0.4, shine, 0.0, 0.0])
    n = 0
    for face in feat.GetFaces() or []:
        cast(face, "IFace2").MaterialPropertyValues = vals
        n += 1
    return n


Part.text = _text
Part.color_faces = _color_faces


# ---------- tight geometry measurement (assemblies) ----------
def comp_matrix(c):
    """4x4 world transform (mm) of an IComponent2."""
    import numpy as np
    d = list(cast(c.Transform2, "IMathTransform").ArrayData)
    M = np.eye(4)
    M[:3, :3] = np.array(d[:9]).reshape(3, 3).T * d[12]
    M[:3, 3] = np.array(d[9:12]) / MM
    return M


def comp_bodies(c):
    bodies = c.GetBodies3(0, None)          # swSolidBody
    if isinstance(bodies, tuple) and len(bodies) == 2 and not hasattr(bodies[0], "GetExtremePoint"):
        bodies = bodies[0]
    return [cast(b, "IBody2") for b in (bodies or [])]


def tight_box(comps):
    """Exact axis-aligned box (mm) of the given components' solid bodies, via IBody2.GetExtremePoint."""
    import numpy as np
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    per = {}
    for c in comps:
        c = cast(c, "IComponent2")
        if c.IsSuppressed() or not c.Visible:
            continue
        M = comp_matrix(c)
        R, t = M[:3, :3], M[:3, 3]
        clo, chi = np.full(3, np.inf), np.full(3, -np.inf)
        for b in comp_bodies(c):
            for ax in range(3):
                for s in (1, -1):
                    dw = np.zeros(3); dw[ax] = s
                    dl = R.T @ dw
                    r = b.GetExtremePoint(dl[0], dl[1], dl[2])
                    ok, x, y, z = r if len(r) == 4 else (True,) + tuple(r[-3:])
                    if not ok:
                        continue
                    pw = R @ (np.array([x, y, z]) / MM) + t
                    clo[ax] = min(clo[ax], pw[ax]); chi[ax] = max(chi[ax], pw[ax])
        if np.isfinite(clo).all():
            per[c.Name2] = (clo.round(2).tolist(), chi.round(2).tolist())
            lo, hi = np.minimum(lo, clo), np.maximum(hi, chi)
    return lo.round(2).tolist(), hi.round(2).tolist(), per


def zoom_to(m, comp, factor=0.75):
    """Zoom the active view onto one component, then back off by `factor` (<1 shows more context)."""
    m.ClearSelection2(True)
    comp.Select4(False, None, False)
    m.ViewZoomToSelection()
    m.ClearSelection2(True)
    try:
        cast(m.ActiveView, "IModelView").ZoomByFactor(factor)
    except Exception:
        pass
    m.GraphicsRedraw2()
