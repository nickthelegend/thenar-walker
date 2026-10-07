"""Estimate P1S print time and filament for every rover plate from the real geometry.

No slicer is installed on this PC, so this slices each part at 0.2 mm like a slicer does
(perimeter walls, solid top/bottom skins, sparse infill) and converts extruded volume to time
with Bambu P1S flow limits. Expect +-25 %; Bambu Studio shows the exact time after slicing.
  python cad/tools/estimate_print_time.py   -> cad/print/print_time.json
"""
import json
import os

import numpy as np
import trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union

import design as D
import make_plates as MP

H = 0.2            # layer height
W = 0.45           # line width
PROFILE = {        # walls, top+bottom solid layers, sparse infill fraction, effective mm^3/s (walls, solid, sparse), s per layer
    'PETG': dict(walls=4, top=5, bottom=4, infill=0.30, rate=(8.0, 9.5, 11.0), layer_s=3.0, density=1.27),
    'TPU 95A': dict(walls=3, top=4, bottom=3, infill=0.20, rate=(2.6, 2.9, 3.2), layer_s=4.0, density=1.21),
}
PLATE_OVERHEAD_S = 6 * 60      # heat bed/nozzle, auto bed level, purge line, cool-down


def layers(mesh):
    zs = np.arange(H / 2, mesh.bounds[1][2], H)
    secs = mesh.section_multiplane(plane_origin=[0, 0, 0], plane_normal=[0, 0, 1], heights=zs)
    out = []
    for s in secs:
        if s is None:
            out.append(Polygon())
            continue
        polys = [p for p in s.polygons_full if p is not None and p.is_valid]
        out.append(unary_union(polys) if polys else Polygon())
    return out


def part_volumes(mesh, prof):
    L = layers(mesh)
    n = len(L)
    walls = solid = sparse = 0.0
    for i, poly in enumerate(L):
        if poly.is_empty:
            continue
        perim = poly.length
        wall_area = min(poly.area, prof['walls'] * W * perim)
        inner = poly.buffer(-prof['walls'] * W)
        # solid skin: what is not covered by the next `top` layers above or the `bottom` layers below
        above = [L[j] for j in range(i + 1, min(n, i + 1 + prof['top']))]
        below = [L[j] for j in range(max(0, i - prof['bottom']), i)]
        cover_up = above[0] if len(above) == prof['top'] else Polygon()
        for a in above[1:]:
            cover_up = cover_up.intersection(a)
        cover_dn = below[0] if len(below) == prof['bottom'] else Polygon()
        for b in below[1:]:
            cover_dn = cover_dn.intersection(b)
        if len(above) < prof['top']:
            cover_up = Polygon()
        if len(below) < prof['bottom']:
            cover_dn = Polygon()
        inner_area = max(inner.area, 0.0) if not inner.is_empty else 0.0
        solid_area = 0.0
        if inner_area > 0:
            solid_region = inner.difference(cover_up.intersection(cover_dn)) if not (cover_up.is_empty or cover_dn.is_empty) else inner
            solid_area = solid_region.area
        sparse_area = max(inner_area - solid_area, 0.0)
        walls += wall_area * H
        solid += solid_area * H
        sparse += sparse_area * H * prof['infill']
    return walls, solid, sparse, n


def main():
    plates = json.load(open(os.path.join(D.CAD, 'print', 'plates.json')))
    cache = {}
    rows = []
    for pl in plates['plates']:
        mat = pl['material']
        prof = PROFILE[mat]
        t = PLATE_OVERHEAD_S
        vol = 0.0
        max_layers = 0
        for stl in pl['parts']:
            key = (stl, mat)
            if key not in cache:
                m = MP.load(stl)
                cache[key] = part_volumes(m, prof)
            w, s, sp, n = cache[key]
            vol += w + s + sp
            t += w / prof['rate'][0] + s / prof['rate'][1] + sp / prof['rate'][2]
            max_layers = max(max_layers, n)
        t += max_layers * prof['layer_s'] * len(pl['parts']) ** 0.5   # travel / layer change on a multi-part plate
        grams = vol / 1000 * prof['density']
        rows.append({'plate': pl['file'], 'material': mat, 'est_hours': round(t / 3600, 1), 'est_filament_g': round(grams)})
        print(f"{pl['file']:52s} {mat:8s} ~{t / 3600:4.1f} h   ~{grams:4.0f} g", flush=True)
    tot_h = sum(r['est_hours'] for r in rows)
    tot_petg = sum(r['est_filament_g'] for r in rows if r['material'] == 'PETG')
    tot_tpu = sum(r['est_filament_g'] for r in rows if r['material'] != 'PETG')
    print(f"\nTOTAL ~{tot_h:.1f} h of printing, ~{tot_petg} g PETG + ~{tot_tpu} g TPU (estimate +-25 %)")
    json.dump({'method': __doc__.strip().splitlines()[0], 'accuracy': '+-25 %', 'plates': rows,
               'total_hours': round(tot_h, 1), 'petg_g': tot_petg, 'tpu_g': tot_tpu},
              open(os.path.join(D.CAD, 'print', 'print_time.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
