"""Slice rover plates with the installed Bambu Studio (CLI) for a Bambu Lab P1S, 0.4 nozzle.

  python cad/tools/slice_plates.py 3          # plate 3 only
  python cad/tools/slice_plates.py            # all plates
Writes cad/print/sliced/<plate>_sliced.3mf (open it in Bambu Studio -> Print) and prints the
slicer's own time / filament prediction.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

import design as D

BIN = Path(r'D:\Program Files\Bambu Studio\bambu-studio.exe')
RES = BIN.parent / 'resources' / 'profiles' / 'BBL'
PRINT = Path(D.CAD) / 'print'
OUT = PRINT / 'sliced'

FILAMENT = {'PETG': 'Bambu PETG Basic @BBL X1C', 'TPU 95A': 'Bambu TPU 95A @BBL X1C'}
PROCESS_OVERRIDES = {
    'PETG': {'wall_loops': '4', 'sparse_infill_density': '30%', 'sparse_infill_pattern': 'gyroid'},
    'TPU 95A': {'wall_loops': '3', 'sparse_infill_density': '20%', 'sparse_infill_pattern': 'gyroid'},
}


def resolve(name, sub):
    path = RES / sub / (name + '.json')
    data = json.loads(path.read_text(encoding='utf-8'))
    out = {}
    if data.get('inherits'):
        out.update(resolve(data['inherits'], sub))
    for inc in data.get('include', []):
        out.update(resolve(inc, sub))
    out.update(data)
    out.pop('inherits', None)
    out.pop('include', None)
    return out


def write_profiles(folder, material):
    machine = resolve('Bambu Lab P1S 0.4 nozzle', 'machine')
    process = resolve('0.20mm Standard @BBL X1C', 'process')
    filament = resolve(FILAMENT[material], 'filament')
    for d in (machine, process):
        d['curr_bed_type'] = 'Textured PEI Plate'
    process.update(PROCESS_OVERRIDES[material])
    process.update({'name': f'Thenar Walker rover {material} 0.20', 'enable_support': '0', 'brim_type': 'auto_brim',
                    'enable_prime_tower': '0', 'print_sequence': 'by layer'})
    paths = {}
    for k, d in (('machine', machine), ('process', process), ('filament', filament)):
        p = folder / f'{k}.json'
        p.write_text(json.dumps(d, indent=1), encoding='utf-8')
        paths[k] = p
    return paths


def read_result(sliced):
    with zipfile.ZipFile(sliced) as z:
        info = z.read('Metadata/slice_info.config').decode('utf-8', 'replace')
    pred = re.search(r'key="prediction" value="(\d+)"', info)
    weight = re.search(r'key="weight" value="([\d.]+)"', info)
    used = re.findall(r'used_g="([\d.]+)"', info)
    secs = int(pred.group(1)) if pred else None
    return {'time_s': secs, 'time': f'{secs // 3600} h {secs % 3600 // 60} min' if secs else None,
            'weight_g': float(weight.group(1)) if weight else (sum(map(float, used)) if used else None)}


def slice_plate(plate):
    meta = json.load(open(PRINT / 'plates.json'))
    entry = [p for p in meta['plates'] if p['file'].startswith(f'P1S_Rover_{plate}_')][0]
    src = PRINT / entry['file']
    OUT.mkdir(exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix='twslice_'))
    prof = write_profiles(work, entry['material'])
    name = src.stem + '_sliced.3mf'
    args = [str(BIN), '--debug', '3', '--load-settings', f"{prof['machine']};{prof['process']}", '--load-filaments', str(prof['filament']),
            '--arrange', '0', '--orient', '0', '--slice', '0', '--outputdir', str(OUT), '--export-3mf', name, str(src)]
    run = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors='replace', timeout=1800)
    (OUT / (src.stem + '_slicer.log')).write_text(run.stdout, encoding='utf-8')
    out = OUT / name
    if run.returncode != 0 or not out.exists():
        print(run.stdout[-3000:])
        raise SystemExit(f'slicing failed (exit {run.returncode}) — see {OUT / (src.stem + "_slicer.log")}')
    r = read_result(out)
    r.update({'plate': entry['file'], 'material': entry['material'], 'sliced_file': str(out)})
    print(json.dumps(r, indent=1))
    return r


if __name__ == '__main__':
    plates = sys.argv[1:] or ['1', '2', '3', '4', '5']
    results = [slice_plate(p) for p in plates]
    path = OUT / 'slice_results.json'
    old = json.load(open(path)) if path.exists() else {}
    old.update({r['plate']: r for r in results})
    json.dump(old, open(path, 'w'), indent=1)
