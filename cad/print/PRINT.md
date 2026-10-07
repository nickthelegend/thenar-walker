# Rover print plates — Bambu Lab P1S

Five plates, everything for the driving base. (The arm plates are in thenar-arms.)
Open a `.3mf` in Bambu Studio → it loads already placed on the 256 × 256 bed → pick the filament → slice → print.
If Bambu Studio asks "load as a single object with multiple parts?", answer **No**.

| # | File | Material | Parts | Height | Time* | Filament* |
|---|---|---|---|---|---|---|
| 1 | `P1S_Rover_1_Tub_Cage_Bars.3mf` — **tub v2**: mounts for the 370 plastic-gearbox motors (M8 collar + nut pocket), walls 100 high for the 3 mm wooden deck, THENAR.IO / THENARLABS engraved; cage along Y | PETG | chassis tub, battery cage, 2 battery bars | 43 mm | 6 h 51 min (sliced) | 252 g (sliced) |
| 2 | `P1S_Rover_2_Deck.3mf` | PETG | deck plate (250 mm long) + 4 bosses for the Cytron MDD10A | 11 mm | ~3.0 h (Bambu Studio: 3 h 27 min) | ~127 g (sliced: 117 g) |
| 3 | `P1S_Rover_3_FrontWheel_RearHub_A_PRINT_FIRST.3mf` | PETG | 1 front wheel (one piece) + 1 rear hub | 28 mm | ~5.4 h | ~211 g |
| 4 | `P1S_Rover_4_FrontWheel_RearHub_B.3mf` | PETG | 1 front wheel + 1 rear hub | 28 mm | ~5.4 h | ~211 g |
| 4b | `P1S_Rover_4b_FrontWheel_RearHub_6mm_shaft.3mf` — **use instead of plate 4 for the local 370 plastic-gearbox motor** (6 mm round shaft): 6.3 mm round hole + M3 grub | PETG | 1 front wheel + 1 rear hub | 28 mm | 6 h 7 min (sliced) | 202 g (sliced) |
| 5F | `P1S_Rover_5F_FlatTyreStrips_2tyres_TPU.3mf` — **flat alternative to plate 5: 2 tyres** as 4 straight half-strips with a click (puzzle) joint, 2 % short so they sit stretched on the hub; superglue the joints | **TPU 95A** | 4 half-strips = 2 tyres | 6 mm | 5 h 40 min (sliced) | 84 g (sliced) |
| 5G | `P1S_Rover_5G_FlatTyreStrips_4tyres_TPU.3mf` — same strips, **8 half-strips = 4 tyres (2 + 2 spare)** | **TPU 95A** | 8 half-strips | 6 mm | 11 h 13 min (sliced) | 168 g (sliced) |
| 5 | `P1S_Rover_5_RearTyres_TPU.3mf` | **TPU 95A** | 2 rear tyres | 25 mm | ~7.7 h | ~87 g |

\* estimated from the real geometry (`cad/tools/estimate_print_time.py`, ±25 %); Bambu Studio shows the exact numbers after slicing.
**Total ≈ 28.6 h of printing, ≈ 0.81 kg PETG + ≈ 87 g TPU** (rover only — buy a 1 kg PETG spool and a small TPU spool).

## Who prints what

| Printer | Plates | Time | Filament |
|---|---|---|---|
| **Friend** — `Friend_Print_Package_Plates_1_and_5.zip` | 1 (tub + cage + bars), 5 (TPU rear tyres) | ~15 h | ~260 g PETG + ~90 g TPU |
| **Us** | 3 (first!), 4, 2 | ~14 h | ~550 g PETG |

The friend's printer needs a ≥ 256 × 256 mm bed and a direct-drive extruder for TPU.

**Print plate 3 first:** its front wheel is the one you put on the first motor for the 1 L water-bottle test.

## Settings

| | PETG plates (1–4) | TPU plate (5) |
|---|---|---|
| Profile | Bambu PETG Basic / generic PETG, 0.20 mm Standard | Bambu TPU 95A / generic TPU, 0.20 mm |
| Walls | 4 | 3 |
| Infill | 30 % gyroid | 20 % gyroid |
| Supports | **none** (checked: only short bridges and ≤ 1.5 mm ledges) | none |
| Plate | textured PEI (glue stick on smooth PEI) | textured PEI |
| Notes | tub: add a 5 mm brim if corners lift | dry the TPU first; slow (max volumetric ~3.6 mm³/s) |

## Which way up (already set in the files)

| Part | On the bed |
|---|---|
| RR-01 tub | floor down, walls up — motor U-slots, vent slots and USB hole print as short bridges |
| RR-02 deck | flat underside down, arm fence and the 4 MDD10A bosses up (the ESP32 now hangs on 2 brass M2.5 × 10 standoffs, so nothing sticks out below) |
| RR-03 cage / RR-04 bars | open box up / flat |
| RR-05 rear hub | spoke disc (inner face) down |
| RR-07 front wheel | inner face down (one piece: hub + hard tread) |
| RR-06 rear tyre | ring standing on its edge |

## After printing

1. Heat-set 4 × M3 inserts into the tub's corner columns.
2. Clean the 4 mm D-bore of every wheel with a 4 mm drill by hand if the shaft is tight.
3. Tap the grub-screw holes (Ø2.6) with an M3 tap, or let the M3 grub screw cut its own thread.
4. Stretch the TPU tyres onto the two rear hubs (TPU = rear, hard one-piece wheels = front).

Single-part STLs for re-arranging are in `cad/stl/`. Rebuild the plates after any CAD change:
`python cad/tools/build_parts.py` then `python cad/tools/make_plates.py`.
