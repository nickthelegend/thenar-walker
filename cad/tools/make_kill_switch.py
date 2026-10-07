"""KCD4 rocker kill switch (16 A 250 VAC / 20 A 125 VAC, the team's switch) as a detailed mesh.

Standard KCD4 DPST dimensions (the switch itself was not available to measure):
  frame 25 × 31.5 × 2.5 above the panel, rocker 18.4 × 25.4 rising 4–8 mm above the frame (ON side up),
  body 21.6 × 29.6 × 23 deep, snap clips on the long sides, 4 × 6.3 mm quick-connect tabs 9 mm long.
  Panel cut-out 22.2 × 30.2 (design.KILL_CUT), panels 1–3 mm (the wooden deck is 3 mm).

Local frame: z = 0 is the top of the panel, +z out of the deck, x across the short side, y along the long side.
  python cad/tools/make_kill_switch.py   -> cad/stl/P-08_Kill_Switch_KCD4.STL (all parts) + parts dict for the GLB
"""
import os

import numpy as np
import trimesh
from shapely.geometry import Polygon

import design as D

STL = os.path.join(D.CAD, 'stl')
FX, FY, FT = 25.0, 31.5, 2.5            # frame
OX, OY = 19.4, 26.4                      # frame opening
RX, RY = 18.4, 25.4                      # rocker
BX, BY, BD = 21.6, 29.6, 23.0            # body below the panel
TAB_W, TAB_T, TAB_L = 6.3, 0.8, 9.0      # quick-connect tabs


def box(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])


def rocker():
    """Wedge: top surface tilted along y (ON end high), slightly rounded by two facets."""
    prof = Polygon([(-RY / 2, 0.0), (RY / 2, 0.0), (RY / 2, 8.2), (RY / 2 - 3, 8.6), (2.0, 6.4), (-RY / 2 + 3, 4.6), (-RY / 2, 4.2)])
    m = trimesh.creation.extrude_polygon(prof, RX)            # profile in (y, z), extruded along +x_local
    # extrude_polygon: polygon in XY, extrusion along Z -> map (X, Y, Z) -> (Z - RX/2, X, Y + FT - 1)
    M = np.array([[0, 0, 1, -RX / 2], [1, 0, 0, 0], [0, 1, 0, FT - 1.0], [0, 0, 0, 1]], float)
    m.apply_transform(M)
    return m


def parts():
    frame = trimesh.boolean.difference([box(-FX / 2, FX / 2, -FY / 2, FY / 2, 0, FT),
                                        box(-OX / 2, OX / 2, -OY / 2, OY / 2, -1, FT + 1)], engine='manifold')
    body = box(-BX / 2, BX / 2, -BY / 2, BY / 2, -BD, 0)
    clips = []
    for s in (1, -1):                                         # spring clips on the long sides, leaning out
        c = box(-0.4, 0.4, -4.0, 4.0, -13.0, -2.5)
        c.apply_transform(trimesh.transformations.rotation_matrix(np.radians(-8 * s), [0, 1, 0], [0, 0, -13.0]))
        c.apply_translation([s * (BX / 2 + 0.6), 0, 0])
        clips.append(c)
    tabs = []
    for x in (-5.0, 5.0):                                     # 2 poles x 2 contacts (DPST): pole = same x
        for y in (-6.5, 6.5):
            tabs.append(box(x - TAB_T / 2, x + TAB_T / 2, y - TAB_W / 2, y + TAB_W / 2, -BD - TAB_L, -BD))
            hole = trimesh.creation.cylinder(radius=0.8, height=2, sections=16)
            hole.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
            hole.apply_translation([x, y, -BD - TAB_L + 3.0])
            tabs[-1] = trimesh.boolean.difference([tabs[-1], hole], engine='manifold')
    return {'kill_switch_frame': (trimesh.util.concatenate([frame, body] + clips), 'switch_black'),
            'kill_switch_rocker': (rocker(), 'switch_red'),
            'kill_switch_tabs': (trimesh.util.concatenate(tabs), 'steel')}


def main():
    ps = parts()
    allm = trimesh.util.concatenate([m for m, _ in ps.values()])
    out = os.path.join(STL, 'P-08_Kill_Switch_KCD4.STL')
    allm.export(out)
    lo, hi = allm.bounds
    print(f'KCD4: {hi[0] - lo[0]:.1f} x {hi[1] - lo[1]:.1f} mm, from {lo[2]:.1f} to {hi[2]:.1f} mm around the panel top -> {out}')


if __name__ == '__main__':
    main()
