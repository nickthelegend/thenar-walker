"""Cut the team logos (design.LOGOS) 1 mm deep into the outer faces of the tub end walls, in the exported STL.

  python cad/tools/build_parts.py RR-01      # SolidWorks: plain tub -> cad/stl/RR-01_Chassis_Tub.STL
  python cad/tools/engrave_logos.py          # -> keeps the plain copy as RR-01_Chassis_Tub_plain.STL,
                                             #    writes the engraved RR-01_Chassis_Tub.STL (used by make_plates)
Text reads correctly from outside: THENAR.IO on the front wall, THENARLABS on the rear wall (Arial Black).
"""
import math
import os
import shutil

import numpy as np
import trimesh
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath
from shapely.geometry import Polygon

import design as D

STL = os.path.join(D.CAD, 'stl')
TUB = os.path.join(STL, 'RR-01_Chassis_Tub.STL')
PLAIN = os.path.join(STL, 'RR-01_Chassis_Tub_plain.STL')
FONTS = os.path.join(os.environ.get('WINDIR', 'C:/Windows'), 'Fonts')
FONT = FontProperties(fname=next(os.path.join(FONTS, f) for f in ('ariblk.ttf', 'seguibl.ttf', 'arialbd.ttf')
                                 if os.path.exists(os.path.join(FONTS, f))))


def text_shape(text, cap):
    """Lettering as shapely polygons (mm), centred, x = reading direction, y = up."""
    tp = TextPath((0, 0), text, size=cap / 0.716, prop=FONT)
    bb = tp.get_extents()
    shape = None
    for poly in tp.to_polygons(closed_only=True):
        p = Polygon(np.asarray(poly) - [(bb.x0 + bb.x1) / 2, (bb.y0 + bb.y1) / 2]).buffer(0)
        shape = p if shape is None else shape.symmetric_difference(p)     # even-odd: holes in O, A, R, B
    return shape


def logo_solid(text, wall, yc, zc, cap, depth):
    shape = text_shape(text, cap)
    geoms = list(getattr(shape, 'geoms', [shape]))                   # one prism per letter piece
    prism = trimesh.util.concatenate([trimesh.creation.extrude_polygon(g, depth + 1.0) for g in geoms])   # extrusion along +z_t
    # text frame -> robot frame. A person facing the front wall has the robot's left (+Y) on their right,
    # so the front wall reads toward +Y; facing the rear wall it reads toward -Y.
    s = 1.0 if wall == 'front' else -1.0
    x_face = D.TUB_X if wall == 'front' else -D.TUB_X
    # extrusion axis points into the wall: front wall -> -X, rear wall -> +X; start 1 mm outside the face
    ex = -1.0 if wall == 'front' else 1.0
    M = np.array([[0, 0, ex, x_face - ex * 1.0],
                  [s, 0, 0, yc],
                  [0, 1, 0, zc],
                  [0, 0, 0, 1]], float)
    prism.apply_transform(M)
    return prism


def main():
    if not os.path.exists(PLAIN) or os.path.getmtime(TUB) > os.path.getmtime(PLAIN) + 1:
        shutil.copy2(TUB, PLAIN)                         # a fresh SolidWorks export is the new plain tub
    tub = trimesh.load(PLAIN)
    # the front wall had 5 vent slots until 2026-10-07 (build_parts no longer cuts them). If this export
    # still has them (SolidWorks not re-run yet), fill them so THENAR.IO sits on a solid wall.
    if not tub.contains([[D.TUB_X - D.WALL_END_T / 2, 0.0, 78.0]])[0]:
        x0, x1 = D.TUB_X - D.WALL_END_T, D.TUB_X
        plugs = [trimesh.creation.box(bounds=[[x0, -50 + k * 25 - 4.05, 65.95], [x1, -50 + k * 25 + 4.05, 90.05]]) for k in range(5)]
        tub = trimesh.boolean.union([tub] + plugs, engine='manifold')
        print('filled the 5 old front vent slots (this STL predates their removal)')
    cutters = [logo_solid(t, w, y, z, c, D.LOGO_DEPTH) for t, w, y, z, c in D.LOGOS]
    out = trimesh.boolean.difference([tub] + cutters, engine='manifold')
    assert out.is_watertight
    removed = tub.volume - out.volume
    for (t, w, y, z, c), m in zip(D.LOGOS, cutters):
        lo, hi = m.bounds
        print(f'{t:11s} {w:5s} wall: {hi[1] - lo[1]:.1f} mm long, {hi[2] - lo[2]:.1f} mm tall, z {lo[2]:.1f}..{hi[2]:.1f}')
    print(f'removed {removed:.0f} mm3 (~ {removed / (sum(c.volume for c in cutters) / (D.LOGO_DEPTH + 1) * D.LOGO_DEPTH) * 100:.0f} % of the letter area x depth)')
    out.export(TUB)
    print('->', TUB)


if __name__ == '__main__':
    main()
