"""Flat TPU tyre strips (RR-06F): the rear tyre printed as two straight halves that click together
with a puzzle joint, instead of one ring. A whole strip (~41 cm) does not fit the 256 mm bed, so each
tyre = 2 identical halves; 4 tyres = 8 halves on one P1S plate.

Same cross-section as the ring tyre RR-06 (25 mm wide between the hub bead lips, 6 mm thick, two
1.2 mm tread grooves, 1 mm edge chamfers). The strip is 2 % shorter than the tyre's neutral
circumference so it is stretched tight on the hub once joined (drive torque goes through friction).

  python cad/tools/make_flat_tyres.py   -> cad/stl/RR-06F_Tyre_Strip_Half.STL
"""
import math
import os

import numpy as np
import trimesh
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

import design as D

STL = os.path.join(D.CAD, 'stl')

T = D.TYRE_T                                   # 6 mm radial thickness
WID = D.WHEEL_W - 3.0                          # 25 mm between the bead lips
R_NEUTRAL = D.WHEEL_D / 2 - T / 2              # 67
PRESTRETCH = 0.02
HALF = math.pi * R_NEUTRAL * (1 - PRESTRETCH)  # body length of one half (joint line to joint line)
NECK_L, NECK_W, KNOB_R, CLEAR = 4.0, 9.0, 7.0, 0.15   # big knob: holds the ~80 N hoop tension (glue it too)
GROOVES = [(7.5, 9.5), (15.5, 17.5)]           # tread grooves across the width (y), 1.2 mm deep
GROOVE_D = 1.2
KEY = (2.1, 2.2)                               # grub-screw key hole: y from the chassis-side edge, radius


def outline():
    yc = WID / 2
    body = box(0, 0, HALF, WID)
    male = unary_union([box(HALF - 0.5, yc - NECK_W / 2, HALF + NECK_L + KNOB_R, yc + NECK_W / 2),
                        Point(HALF + NECK_L + KNOB_R, yc).buffer(KNOB_R, 64)])
    female = unary_union([box(-1, yc - NECK_W / 2 - CLEAR, NECK_L + KNOB_R, yc + NECK_W / 2 + CLEAR),
                          Point(NECK_L + KNOB_R, yc).buffer(KNOB_R + CLEAR, 64)])
    return body.union(male).difference(female)


def prism_x(tri_yz, x0, x1):
    """Extrude a triangle given in (y, z) along X from x0 to x1."""
    m = trimesh.creation.extrude_polygon(Polygon(tri_yz), x1 - x0)   # polygon in XY, extruded along +Z
    # map (X, Y, Z) -> (Z + x0, X, Y): polygon x->y, y->z, extrusion->x
    M = np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)
    m.apply_transform(M)
    return m


def build():
    strip = trimesh.creation.extrude_polygon(outline(), T)            # inner face on the bed, tread up
    x0, x1 = -2, HALF + NECK_L + 2 * KNOB_R + 2
    cuts = [trimesh.creation.box(bounds=[[x0, a, T - GROOVE_D], [x1, b, T + 1]]) for a, b in GROOVES]
    cuts += [prism_x([(-1, T + 1), (1, T + 1), (-1, T - 1)], x0, x1),            # edge chamfers 1 x 1
             prism_x([(WID + 1, T + 1), (WID - 1, T + 1), (WID + 1, T - 1)], x0, x1)]
    key = trimesh.creation.cylinder(radius=KEY[1], height=T + 2, sections=48)
    key.apply_translation([HALF / 2, KEY[0], T / 2])
    cuts.append(key)
    m = trimesh.boolean.difference([strip] + cuts, engine='manifold')
    m.apply_translation([0, 0, -m.bounds[0][2]])
    return m


def check(m):
    assert m.is_watertight, 'strip mesh not watertight'
    ext = m.extents
    assert ext[0] <= 251 and ext[1] <= WID + 0.01 and abs(ext[2] - T) < 0.01, ext
    full = 2 * HALF
    print(f'half strip {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm, volume {m.volume / 1000:.1f} cm3; '
          f'joined tyre {full:.1f} mm = {100 * (1 - full / (2 * math.pi * R_NEUTRAL)):.1f} % short '
          f'(stretched to the {2 * math.pi * R_NEUTRAL:.1f} mm neutral circumference on the hub)')


if __name__ == '__main__':
    m = build()
    check(m)
    out = os.path.join(STL, 'RR-06F_Tyre_Strip_Half.STL')
    m.export(out)
    print('->', out)
