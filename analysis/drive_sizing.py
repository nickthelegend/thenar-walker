"""Drive + power sizing for Thenar Walker (numbers quoted in docs/DESIGN.md).

Motor data: 25GA-370 / JGA25-370 12 V 103:1 (60 rpm no-load, 46 rpm rated): rated 2.7 kg.cm,
stall 8.2 kg.cm, rated 0.30 A, stall 1.3 A (Kitsguru / The Engineer Store listing).
Mass: SolidWorks verification (cad/evidence/verification.json) + 10 % margin.
"""
import json, math, os
HERE = os.path.dirname(__file__)
ver = json.load(open(os.path.join(HERE, '..', 'cad', 'evidence', 'verification.json')))
m = ver['configs']['STOW']['mass_total_g'] / 1000 * 1.10       # kg, +10 % (cube, cables, glue-up)
g, r = 9.81, 0.070
crr, accel = 0.05, 0.5                                            # rolling resistance on ply/TPU, launch accel m/s^2
th = math.radians(15)
F = m * g * (math.sin(th) + crr * math.cos(th)) + m * accel
T_total = F * r                                                   # N.m at the wheels
KGCM = 0.0980665
rated, stall = 2.7, 8.2
per4 = T_total / 4 / KGCM
per2 = T_total / 2 / KGCM                                         # one axle unloaded on a speed breaker
v_rated = 46 / 60 * math.pi * 2 * r
v_free = 60 / 60 * math.pi * 2 * r
# power budget (battery side, 3S LiPo 11.1 V nominal)
I_motor_avg, I_motor_peak = 4 * 0.30, 4 * 1.3
servo_W_avg, servo_W_peak = 6 * 0.9 * 6.0 * 0.5, 6 * 1.4 * 6.0       # avg ~0.45 A each, TowerPro stall 1.4 A
I_servo_avg, I_servo_peak = servo_W_avg / 0.88 / 11.1, servo_W_peak / 0.88 / 11.1
I_logic = 0.25
I_avg = I_motor_avg + I_servo_avg + I_logic
I_peak = I_motor_peak + I_servo_peak + I_logic
cap = 2.2
out = {
    'mass_for_sizing_kg': round(m, 2), 'ramp_force_N': round(F, 2), 'wheel_torque_total_Nm': round(T_total, 3),
    'per_motor_kgcm_4wd': round(per4, 2), 'per_motor_kgcm_2_driving': round(per2, 2),
    'rated_kgcm': rated, 'stall_kgcm': stall,
    'margin_4wd_vs_rated': round(rated / per4, 2), 'margin_2wd_vs_stall': round(stall / per2, 2),
    'speed_rated_m_s': round(v_rated, 2), 'speed_free_m_s': round(v_free, 2),
    'battery_current_avg_A': round(I_avg, 2), 'battery_current_peak_A': round(I_peak, 2),
    'runtime_avg_min_2200mAh_80pct': round(cap * 0.8 / I_avg * 60, 0),
    'servo_6V_peak_A': round(6 * 1.4, 1), 'fuse_A': 15,
}
for k, v in out.items():
    print(f'{k:34s} {v}')
json.dump(out, open(os.path.join(HERE, 'drive_sizing.json'), 'w'), indent=1)
