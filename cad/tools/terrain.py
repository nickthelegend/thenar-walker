"""RoboReach obstacle profile (side view) and the robot's resting pose on it.

Profile along the drive direction X (mm), from the rulebook: 15 deg ramp up to 80 mm,
400 mm bridge, 15 deg ramp down, then speed breakers (half-cylinders R40, peak 40 mm)
'immediately' after. The robot pose for a given rear-axle position is solved so both
axles' tyres (R70) are tangent to the profile without penetrating it.
"""
import math

import numpy as np

import design as D

R_TYRE = D.WHEEL_D / 2
H = 80.0
RUN = H / math.tan(math.radians(D.RULES['ramp_deg']))   # 298.6
X_UP = 0.0
X_TOP0 = X_UP + RUN
X_TOP1 = X_TOP0 + 400.0
X_DOWN = X_TOP1 + RUN
BREAKERS = [X_DOWN + 60 + 100 * i for i in range(4)]      # centres, R40, 100 mm pitch (~39 cm zone)
R_BREAK = D.RULES['speed_breaker_peak']


def height(x):
    x = np.asarray(x, float)
    h = np.where(x < X_UP, 0.0,
        np.where(x < X_TOP0, (x - X_UP) * H / RUN,
        np.where(x < X_TOP1, H,
        np.where(x < X_DOWN, H - (x - X_TOP1) * H / RUN, 0.0))))
    for c in BREAKERS:
        d = np.clip(R_BREAK ** 2 - (x - c) ** 2, 0, None)
        h = np.maximum(h, np.sqrt(d))
    return h


XS = np.arange(-600, X_DOWN + 1100, 0.5)
ZS = height(XS)


def wheel_z(xc):
    """Lowest centre height of a R70 circle at x=xc that does not penetrate the profile."""
    m = np.abs(XS - xc) <= R_TYRE
    dx = XS[m] - xc
    return float(np.max(ZS[m] + np.sqrt(np.clip(R_TYRE ** 2 - dx ** 2, 0, None))))


def pose_for_rear(x_rear):
    """Robot pose with the rear axle centre at x_rear: returns (x_centre, z_centre_of_axles, pitch_deg).

    Front axle lies on the circle of radius = wheelbase around the rear axle; pick the
    lowest pitch at which the front tyre clears the profile (it then rests on it)."""
    wb = 2 * D.AXLE_X
    zr = wheel_z(x_rear)
    best = None
    for th in np.radians(np.arange(-40, 40.001, 0.02)):
        xf, zf = x_rear + wb * math.cos(th), zr + wb * math.sin(th)
        if zf >= wheel_z(xf) - 1e-6:
            best = th
            break
    xf, zf = x_rear + wb * math.cos(best), zr + wb * math.sin(best)
    return (x_rear + xf) / 2, (zr + zf) / 2, math.degrees(best)


def robot_transform(x_rear):
    """Robot-frame -> terrain-frame rotation (about Y, nose up = positive pitch) and translation (mm)."""
    xc, zc, pitch = pose_for_rear(x_rear)
    th = math.radians(pitch)
    # robot X forward tilts up by th: rotation about -Y in a right-handed Z-up frame
    R = np.array([[math.cos(th), 0, -math.sin(th)], [0, 1, 0], [math.sin(th), 0, math.cos(th)]])
    axle_mid = np.array([xc, 0, zc])
    t = axle_mid - R @ np.array([0, 0, D.AXLE_Z])
    return R, t, pitch


CASES = {   # name: rear-axle x
    'ramp_foot': X_UP - D.AXLE_X - 20,
    'on_ramp': X_UP + 120,
    'ramp_crest': X_TOP0 - 40,
    'bridge': X_TOP0 + 200,
    'bridge_exit': X_TOP1 - 30,
    'down_ramp_foot': X_DOWN - 80,
    'front_on_breaker_1': BREAKERS[0] - 2 * D.AXLE_X,
    'breaker_under_belly': BREAKERS[1] - D.AXLE_X,
    'rear_on_breaker_4': BREAKERS[3],
}

if __name__ == '__main__':
    for n, xr in CASES.items():
        R, t, p = robot_transform(xr)
        print(f'{n:22s} rear x {xr:7.1f}  pitch {p:6.2f} deg  t {np.round(t, 1)}')
