"""Bambu Lab P1S print plates for every rover part (the arm plates are in thenar-arms).

  python cad/tools/make_plates.py
Writes cad/print/P1S_Rover_<n>_<name>.3mf (+ .stl and a .png preview) and cad/print/plates.json.
Every part is placed flat in its print orientation, checked to fit the 256 x 256 bed with
>= 6 mm between parts, and scanned for overhangs that would need supports.
"""
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from shapely.geometry import MultiPoint

import design as D

STL = os.path.join(D.CAD, 'stl')
OUT = os.path.join(D.CAD, 'print')
BED = 256.0
GAP = 6.0
EDGE = 2.5            # P1S prints the full 256 x 256; parts are also kept >= 10 mm off the front (purge line) edge

# part -> (stl name, material, why this orientation)
PARTS = {
    'tub': ('RR-01_Chassis_Tub', 'PETG', 'floor on the bed, walls up'),
    'deck': ('RR-02_Deck_Plate', 'PETG', 'flat underside on the bed, fence up'),
    'cage': ('RR-03_Battery_Cradle', 'PETG', 'open box, floor tabs on the bed'),
    'bar': ('RR-04_Battery_Bar', 'PETG', 'flat'),
    'hub': ('RR-05_Wheel_Hub', 'PETG', 'web (inner face) on the bed'),
    'front_wheel': ('RR-07_Front_Wheel_PETG', 'PETG', 'one piece, inner face on the bed'),
    'tyre_tpu': ('RR-06_Wheel_Tyre', 'TPU 95A', 'ring standing on its edge'),
    'tyre_strip': ('RR-06F_Tyre_Strip_Half', 'TPU 95A', 'flat strip, inner face on the bed, tread up'),
    'hub_6mm': ('RR-05_Wheel_Hub_6mm', 'PETG', 'web (inner face) on the bed'),
    'front_wheel_6mm': ('RR-07_Front_Wheel_PETG_6mm', 'PETG', 'one piece, inner face on the bed'),
}

# plate name, material, [(part, centre x, centre y)] — centres in bed mm (origin front-left)
PLATES = [
    # tub v2 (cad/tools/tub_v2.py): battery cage now runs along Y -> printed turned 90°
    ('1_Tub_Cage_Bars', 'PETG', [('tub', 128, 100), ('cage', 128, 213, 90), ('bar', 35, 213), ('bar', 221, 213)]),   # tub clear of the P1S front-left exclusion zone
    ('2_Deck', 'PETG', [('deck', 128, 128)]),
    ('3_FrontWheel_RearHub_A_PRINT_FIRST', 'PETG', [('front_wheel', 72.5, 80.5), ('hub', 187.5, 187.5)]),
    ('4_FrontWheel_RearHub_B', 'PETG', [('front_wheel', 72.5, 80.5), ('hub', 187.5, 187.5)]),
    # same as plate 4 but for the local 370 plastic-gearbox motor (6 mm round shaft)
    ('4b_FrontWheel_RearHub_6mm_shaft', 'PETG', [('front_wheel_6mm', 72.5, 80.5), ('hub_6mm', 187.5, 187.5)]),
    # flat alternative to plate 5: 4 straight half-strips = 2 tyres (make_flat_tyres.py)
    ('5F_FlatTyreStrips_2tyres_TPU', 'TPU 95A', [('tyre_strip', 133, 60 + 45 * k) for k in range(4)]),
    # 8 half-strips = 4 tyres (2 + 2 spare) on one plate
    ('5G_FlatTyreStrips_4tyres_TPU', 'TPU 95A', [('tyre_strip', 133, 22.5 + 31 * k) for k in range(8)]),   # x >= 20: clear of the P1S front-left exclusion zone   # x >= 20: clear of the P1S front-left exclusion zone
    ('5_RearTyres_TPU', 'TPU 95A', [('tyre_tpu', 72.5, 80.5), ('tyre_tpu', 183.5, 183.5)]),
]

SETTINGS = {
    'PETG': 'Bambu PETG / generic PETG, 0.2 mm layers, 4 walls, 30 % gyroid, textured PEI plate, no supports, brim off (tub: 5 mm brim if corners lift)',
    'TPU 95A': 'TPU 95A, 0.2 mm layers, 3 walls, 20 % gyroid, max 3.6 mm^3/s (~30 mm/s), no supports, dry the filament first',
}


def load(stl_name):
    m = trimesh.load(os.path.join(STL, stl_name + '.STL'), force='mesh')
    m.apply_translation(-m.bounds[0])            # min corner at origin, so z = 0 is the bed
    m.apply_translation([-m.extents[0] / 2, -m.extents[1] / 2, 0])   # centred in XY
    return m


def overhang_report(m, max_angle=50.0, ignore_below=0.6):
    """Area of downward faces steeper than max_angle from vertical that are not on the bed."""
    n = m.face_normals
    c = m.triangles_center
    down = n[:, 2] < -np.cos(np.radians(max_angle))
    off_bed = c[:, 2] > ignore_below
    idx = np.where(down & off_bed)[0]
    area = float(m.area_faces[idx].sum())
    worst = float(m.area_faces[idx].max()) if len(idx) else 0.0
    return area, worst, idx


def main():
    os.makedirs(OUT, exist_ok=True)
    cache, report = {}, {'bed_mm': BED, 'plates': [], 'parts': {}}
    for key, (name, mat, why) in PARTS.items():
        m = load(name)
        cache[key] = m
        area, worst, _ = overhang_report(m)
        report['parts'][key] = {'stl': name + '.STL', 'material': mat, 'orientation': why,
                                'size_mm': [round(float(v), 1) for v in m.extents],
                                'volume_cm3': round(float(m.volume) / 1000, 1),
                                'unsupported_overhang_mm2': round(area, 1), 'largest_overhang_face_mm2': round(worst, 1)}
    for pname, mat, items in PLATES:
        scene = trimesh.Scene()
        hulls, meshes = [], []
        for i, it in enumerate(items):
            key, cx, cy = it[:3]
            m = cache[key].copy()
            if len(it) > 3 and it[3]:                         # optional rotation about Z (deg), then re-centre
                m.apply_transform(trimesh.transformations.rotation_matrix(np.radians(it[3]), [0, 0, 1]))
                m.apply_translation([-(m.bounds[0][0] + m.bounds[1][0]) / 2, -(m.bounds[0][1] + m.bounds[1][1]) / 2, 0])
            m.apply_translation([cx, cy, 0])
            lo, hi = m.bounds
            assert lo[1] >= 10 - 1e-6, (pname, key, 'too close to the purge line', lo)
            assert lo[0] >= EDGE - 1e-6 and lo[1] >= EDGE - 1e-6 and hi[0] <= BED - EDGE + 1e-6 and hi[1] <= BED - EDGE + 1e-6, \
                (pname, key, lo, hi)
            hulls.append((key, MultiPoint(m.vertices[:, :2]).convex_hull))
            meshes.append(m)
            scene.add_geometry(m, node_name=f'{PARTS[key][0]}_{i + 1}', geom_name=f'{PARTS[key][0]}_{i + 1}')
        for a in range(len(hulls)):
            for b in range(a + 1, len(hulls)):
                d = hulls[a][1].distance(hulls[b][1])
                assert d >= GAP, (pname, hulls[a][0], hulls[b][0], d)
        base = os.path.join(OUT, f'P1S_Rover_{pname}')
        scene.export(base + '.3mf')
        trimesh.util.concatenate(meshes).export(base + '.stl')
        # preview
        fig, ax = plt.subplots(figsize=(4.2, 4.2), dpi=110)
        ax.add_patch(plt.Rectangle((0, 0), BED, BED, fc='#eef2f7', ec='#12355b', lw=1.5))
        for (key, hull), m in zip(hulls, meshes):
            xs, ys = hull.exterior.xy
            ax.fill(xs, ys, fc='#d9622b' if 'TPU' in PARTS[key][1] else '#12355b', alpha=0.75, ec='k', lw=0.5)
            c = hull.centroid
            ax.text(c.x, c.y, PARTS[key][0].split('_', 1)[1].replace('_', ' '), ha='center', va='center', fontsize=6.5, color='white')
        ax.set_xlim(-5, BED + 5); ax.set_ylim(-5, BED + 5); ax.set_aspect('equal'); ax.axis('off')
        ax.set_title(f'P1S plate {pname.replace("_", " ")} — {mat}', fontsize=8)
        fig.savefig(base + '.png', bbox_inches='tight'); plt.close(fig)
        h = max(float(m.bounds[1][2]) for m in meshes)
        report['plates'].append({'file': os.path.basename(base) + '.3mf', 'material': mat, 'settings': SETTINGS[mat],
                                 'parts': [f'{PARTS[it[0]][0]}' for it in items], 'max_height_mm': round(h, 1),
                                 'min_gap_mm': round(min([hulls[a][1].distance(hulls[b][1]) for a in range(len(hulls))
                                                          for b in range(a + 1, len(hulls))] or [0]), 1),
                                 'filament_g_est': round(sum(m.volume for m in meshes) / 1000 * (1.27 if mat == 'PETG' else 1.21) * 0.55, 0)})
        print(f"{pname:32s} {mat:8s} parts {len(items)}  height {h:5.1f} mm")
    json.dump(report, open(os.path.join(OUT, 'plates.json'), 'w'), indent=1)
    print()
    for k, v in report['parts'].items():
        print(f"{k:10s} size {v['size_mm']}  unsupported overhang {v['unsupported_overhang_mm2']:7.1f} mm2 (largest face {v['largest_overhang_face_mm2']} mm2)")


if __name__ == '__main__':
    main()
