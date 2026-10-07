---
workflow: general-video
flow: automation
storyboard: no
message: "Build the Thenar Walker rover step by step, and wire every connection correctly"
destination: team-laptop
aspect: 1920x1080
language: en
audience: the Thenar Walker build team (RoboReach 2026-27)
length: 240s
angle: assembly-manual
---

## Intent

A full 3D assembly film of the Thenar Walker rover built from the project's real CAD parts (tub v2,
370 gear motors with M8 collar + nut, wheels + TPU tyres, battery cage, boards, 3 mm wooden deck,
Cytron MDD10A, kill switch, thenar-arms follower arm). Each part animates into place, then every
electrical connection draws itself: power path, motors to the driver channels, ESP32 signal pins,
servo leads. User's words: "full assembly video with 3d animations ... and connections to each part
of it, each connections please".

## Customizations

- On-screen labels only (step titles, part names, screw sizes, pin names). No voiceover.
- About 4 minutes, slower pace, close-ups on screws, nuts and pins.

## Notes

- Geometry truth: cad/tools/tub_v2.py (tub v2 layout), cad/tools/design.py, docs/WIRING.md.
- Motor can/cap are estimated (total length 50 mm from the gearbox face).
- No music was requested; the film is silent.
