"""Solve Barry's Limelight mount (and the HIVE offset) from a Camera Calibration log.

Usage:  python limelight_extrinsics.py r0012_camera_calibration_<time>.csv

The Camera Calibration OpMode stops the robot at known poses (odometry, relative to a start whose
heading is exact) and logs every tag's raw Limelight camera-space position. This script averages each
tag at each stop, then fits by least squares:
    camera: FORWARD_IN, LEFT_IN, YAW_DEG, PITCH_DEG, ROLL_DEG, DISTANCE_SCALE   (CameraMount)
    HIVE:   x / y / z offset from its nominal position (one per HIVE seen)
HEIGHT_IN stays as logged: the robot only moves on the floor, so only camera height minus HIVE height
can be found (the HIVE z offset absorbs it). The HIVE offset is relative to where odometry put the
start, so it says how far the HIVE is from the robot's start, not where it is on the field.

The +/- values come from refitting on stops resampled with replacement (bootstrap). That covers
errors shared by all tags seen from one stop, such as the odometry pose at that stop, which the
least-squares formula alone would treat as independent and underestimate.

Plain Python, no numpy. Conventions match CameraMount.java and HiveTagGeometry.java.
"""
import csv
import math
import random
import re
import sys
from collections import defaultdict

M = 39.3701  # inches per meter

# ---------------------------------------------------------------------------------------------
# HiveTagGeometry.java (FTC frame: X toward the audience, Y toward blue, Z up), then FTC -> Pedro
# ---------------------------------------------------------------------------------------------
PIVOT_Z = 43.95
HALF = 42.91 / 2
TAG_FROM_PIVOT = HALF - (9.938 - 2.75)
PIVOT_ABOVE_FLOOR = (PIVOT_Z + HALF * math.sin(math.radians(30)) - 53.5) / math.cos(math.radians(30))
OFFSETS = [-6.5, -2.75, 2.75, 6.5]
ORDER = {  # ids in order of increasing FTC Y, with (red, audience)
    30: ([33, 32, 31, 30], True, False), 34: ([34, 35, 36, 37], True, True),
    38: ([38, 39, 40, 41], False, True), 42: ([45, 44, 43, 42], False, False)}


def cluster_of(tag_id):
    return 30 + 4 * ((tag_id - 30) // 4) if 30 <= tag_id <= 45 else None


def tag_world(tag_id, tilt_deg):
    """Nominal tag center, Pedro (x, y, z) inches. tilt in the alliance convention (blue mirrored)."""
    ids, red, audience = ORDER[cluster_of(tag_id)]
    a = math.radians(tilt_deg if red else -tilt_deg)
    s = 1.0 if audience else -1.0
    p = s * TAG_FROM_PIVOT
    x = p * math.cos(a) + PIVOT_ABOVE_FLOOR * math.sin(a)
    y = (-1.0 if red else 1.0) * 25.5 / 2 + OFFSETS[ids.index(tag_id)]
    z = PIVOT_Z + p * math.sin(a) - PIVOT_ABOVE_FLOOR * math.cos(a)
    return (y + 72.0, 72.0 - x, z)


# ---------------------------------------------------------------------------------------------
# CameraMount.java and its inverse
# ---------------------------------------------------------------------------------------------
def camera_to_robot(cam, v):
    """CameraMount.toRobot: Limelight camera-space (m) -> robot frame (forward, left, up) inches."""
    F, L, H, yaw, pitch, roll, scale = cam
    dx, dy, dz = v
    f, l, u = dz, -dx, -dy
    r = math.radians(roll)
    l, u = l * math.cos(r) - u * math.sin(r), l * math.sin(r) + u * math.cos(r)
    p = math.radians(pitch)
    lf, lu = f * math.cos(p) - u * math.sin(p), f * math.sin(p) + u * math.cos(p)
    y = math.radians(yaw)
    rf, rl = lf * math.cos(y) - l * math.sin(y), lf * math.sin(y) + l * math.cos(y)
    k = M * scale
    return (rf * k + F, rl * k + L, lu * k + H)


def robot_to_camera(cam, rv):
    """Inverse of camera_to_robot."""
    F, L, H, yaw, pitch, roll, scale = cam
    k = M * scale
    rf, rl, lu = (rv[0] - F) / k, (rv[1] - L) / k, (rv[2] - H) / k
    y = math.radians(yaw)
    lf, l = rf * math.cos(y) + rl * math.sin(y), -rf * math.sin(y) + rl * math.cos(y)
    p = math.radians(pitch)
    f, u = lf * math.cos(p) + lu * math.sin(p), -lf * math.sin(p) + lu * math.cos(p)
    r = math.radians(roll)
    l, u = l * math.cos(r) + u * math.sin(r), -l * math.sin(r) + u * math.cos(r)
    return (-l, -u, f)


def world_to_robot(pose, w):
    x, y, h = pose
    dx, dy = w[0] - x, w[1] - y
    return (dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h), w[2])


# ---------------------------------------------------------------------------------------------
# Small linear algebra
# ---------------------------------------------------------------------------------------------
def solve_linear(A, b):
    n = len(b)
    aug = [list(A[i]) + [b[i]] for i in range(n)]
    for i in range(n):
        piv = max(range(i, n), key=lambda r: abs(aug[r][i]))
        aug[i], aug[piv] = aug[piv], aug[i]
        if abs(aug[i][i]) < 1e-12:
            raise ValueError("singular")
        for r in range(n):
            if r != i:
                f = aug[r][i] / aug[i][i]
                aug[r] = [a - f * c for a, c in zip(aug[r], aug[i])]
    return [aug[i][n] / aug[i][i] for i in range(n)]


def invert(A):
    n = len(A)
    cols = []
    for j in range(n):
        e = [0.0] * n
        e[j] = 1.0
        cols.append(solve_linear(A, e))
    return [[cols[j][i] for j in range(n)] for i in range(n)]


# ---------------------------------------------------------------------------------------------
# Log reading
# ---------------------------------------------------------------------------------------------
def f(v):
    try:
        x = float(v)
        return None if math.isnan(x) else x
    except (TypeError, ValueError):
        return None


def read_log(path):
    settings = None
    samples = defaultdict(list)          # (stop, tag) -> [(cam xyz, odo pose)]
    for r in csv.DictReader(open(path)):
        if r['state'] == 'event' and (r['event'] or '').startswith('settings'):
            nums = dict(re.findall(r'(forward|left|height|yaw|pitch|roll|scale) (-?[\d.]+)', r['event']))
            settings = {k: float(v) for k, v in nums.items()}
        if r['state'] != 'COLLECT' or f(r['cam_z_m']) is None:
            continue
        tag = int(float(r['tag_id']))
        if cluster_of(tag) is None:
            continue
        samples[(int(float(r['stop'])), tag)].append((
            (f(r['cam_x_m']), f(r['cam_y_m']), f(r['cam_z_m'])),
            (f(r['odo_x']), f(r['odo_y']), math.radians(f(r['odo_h_deg'])))))
    if settings is None:
        raise SystemExit("No settings row: is this a Camera Calibration log?")
    obs = []
    for (stop, tag), rows in samples.items():
        if len(rows) < 5:
            continue
        n = len(rows)
        cam = tuple(sum(c[i] for c, _ in rows) / n for i in range(3))
        x = sum(o[0] for _, o in rows) / n
        y = sum(o[1] for _, o in rows) / n
        h = math.atan2(sum(math.sin(o[2]) for _, o in rows), sum(math.cos(o[2]) for _, o in rows))
        obs.append(dict(stop=stop, tag=tag, cam=cam, pose=(x, y, h), frames=n))
    return settings, obs


# ---------------------------------------------------------------------------------------------
# Fit
# ---------------------------------------------------------------------------------------------
CAMERA = ['F', 'L', 'yaw', 'pitch', 'roll', 'scale']
LATERAL_SIGMA_IN = 0.3       # expected per-observation noise across the line of sight
DEPTH_SIGMA = 0.01           # ... and along it, as a fraction of the distance


def snap_tilts(obs, cam):
    """Per HIVE (red / blue): +30 or -30, whichever puts the measured tag heights closer."""
    tilts = {}
    for red in (True, False):
        mine = [o for o in obs if (o['tag'] <= 37) == red]
        if not mine:
            continue
        err = {}
        for t in (30.0, -30.0):
            err[t] = sum(abs(camera_to_robot(cam, o['cam'])[2] - tag_world(o['tag'], t)[2]) for o in mine)
        tilts[red] = min(err, key=err.get)
    return tilts


def residuals(params, obs, H, tilts):
    cam = (params['F'], params['L'], H, params['yaw'], params['pitch'], params['roll'], params['scale'])
    out = []
    for o in obs:
        red = o['tag'] <= 37
        w = tag_world(o['tag'], tilts[red])
        key = 'red' if red else 'blue'
        w = (w[0] + params['ex_' + key], w[1] + params['ey_' + key], w[2] + params['ez_' + key])
        pred = robot_to_camera(cam, world_to_robot(o['pose'], w))
        meas = o['cam']
        rng = math.sqrt(sum(v * v for v in meas)) * M
        out += [(pred[0] - meas[0]) * M / LATERAL_SIGMA_IN,
                (pred[1] - meas[1]) * M / LATERAL_SIGMA_IN,
                (pred[2] - meas[2]) * M / (DEPTH_SIGMA * rng)]
    return out


def fit(obs, start, H, tilts, names):
    p = dict(start)
    for _ in range(50):
        r0 = residuals(p, obs, H, tilts)
        J = []
        for n in names:
            q = dict(p)
            q[n] += 1e-5
            J.append([(a - b) / 1e-5 for a, b in zip(residuals(q, obs, H, tilts), r0)])
        A = [[sum(J[i][k] * J[j][k] for k in range(len(r0))) for j in range(len(names))] for i in range(len(names))]
        g = [-sum(J[i][k] * r0[k] for k in range(len(r0))) for i in range(len(names))]
        d = solve_linear(A, g)
        for n, dv in zip(names, d):
            p[n] += dv
        if max(abs(x) for x in d) < 1e-8:
            break
    r = residuals(p, obs, H, tilts)
    chi2 = sum(x * x for x in r) / max(1, len(r) - len(names))
    cov = invert(A)
    sd = {n: math.sqrt(max(0.0, cov[i][i] * chi2)) for i, n in enumerate(names)}
    return p, sd, r


def bootstrap_sd(obs, params, H, tilts, names, rounds=50, seed=1):
    """Spread of each parameter over refits on stops resampled with replacement."""
    rnd = random.Random(seed)
    by_stop = defaultdict(list)
    for o in obs:
        by_stop[o['stop']].append(o)
    stops = list(by_stop)
    fits = []
    for _ in range(rounds):
        sample = [o for s in (rnd.choice(stops) for _ in stops) for o in by_stop[s]]
        try:
            fits.append(fit(sample, params, H, tilts, names)[0])
        except ValueError:
            continue  # a resample without enough variety to solve
    sd = {}
    for n in names:
        vals = [p[n] for p in fits]
        mean = sum(vals) / len(vals)
        sd[n] = math.sqrt(sum((v - mean) ** 2 for v in vals) / max(1, len(vals) - 1))
    return sd


def per_obs_error_in(params, obs, H, tilts):
    cam = (params['F'], params['L'], H, params['yaw'], params['pitch'], params['roll'], params['scale'])
    out = []
    for o in obs:
        red = o['tag'] <= 37
        key = 'red' if red else 'blue'
        w = tag_world(o['tag'], tilts[red])
        w = (w[0] + params['ex_' + key], w[1] + params['ey_' + key], w[2] + params['ez_' + key])
        meas_robot = camera_to_robot(cam, o['cam'])
        true_robot = world_to_robot(o['pose'], w)
        out.append(math.sqrt(sum((a - b) ** 2 for a, b in zip(meas_robot, true_robot))))
    return out


def main(path):
    settings, obs = read_log(path)
    H = settings['height']
    stops = sorted({o['stop'] for o in obs})
    print(f"{path}\n  {len(obs)} tag observations from {len(stops)} stops "
          f"(tags {sorted({o['tag'] for o in obs})}); camera height {H:.3f} in (fixed)")
    before = dict(F=settings['forward'], L=settings['left'], yaw=settings['yaw'], pitch=settings['pitch'],
                  roll=settings['roll'], scale=settings['scale'])
    cam0 = (before['F'], before['L'], H, before['yaw'], before['pitch'], before['roll'], before['scale'])
    tilts = snap_tilts(obs, cam0)
    print("  HIVE tilt: " + ", ".join(f"{'red' if red else 'blue'} {t:+.0f}" for red, t in tilts.items()))

    start = dict(before)
    hives = ['red' if red else 'blue' for red in tilts]
    names = list(CAMERA)
    for h in hives:
        for a in 'xyz':
            start['e%s_%s' % (a, h)] = 0.0
            names.append('e%s_%s' % (a, h))
    for h in ('red', 'blue'):
        for a in 'xyz':
            start.setdefault('e%s_%s' % (a, h), 0.0)

    err0 = per_obs_error_in(start, obs, H, tilts)
    params, sd, _ = fit(obs, start, H, tilts, names)
    err1 = per_obs_error_in(params, obs, H, tilts)

    # Drop gross outliers (e.g. a misdetection) and refit once
    limit = max(3.0, 4.0 * sorted(err1)[len(err1) // 2])
    kept = [o for o, e in zip(obs, err1) if e <= limit]
    if len(kept) < len(obs):
        print(f"  dropped {len(obs) - len(kept)} outlier observations (> {limit:.1f} in), refitting")
        obs = kept
        params, sd, _ = fit(obs, params, H, tilts, names)
        err1 = per_obs_error_in(params, obs, H, tilts)

    sd = bootstrap_sd(obs, params, H, tilts, names)

    def med(v):
        s = sorted(v)
        return s[len(s) // 2]
    print(f"\n  tag position error (camera vs. model, inches): before median {med(err0):.2f}, "
          f"after median {med(err1):.2f} (90% {sorted(err1)[int(len(err1) * .9)]:.2f})")

    labels = dict(F=('FORWARD_IN', 'in'), L=('LEFT_IN', 'in'), yaw=('YAW_DEG', 'deg'), pitch=('PITCH_DEG', 'deg'),
                  roll=('ROLL_DEG', 'deg'), scale=('DISTANCE_SCALE', ''))
    print("\n  camera mount            before      after   +/-")
    for n in CAMERA:
        print(f"    {labels[n][0]:15s} {before[n]:+10.3f} {params[n]:+10.3f} {sd[n]:6.3f} {labels[n][1]}")
    for h in hives:
        print(f"  {h} HIVE offset from nominal, relative to the start: "
              f"x {params['ex_' + h]:+.2f} +/- {sd['ex_' + h]:.2f}, y {params['ey_' + h]:+.2f} +/- {sd['ey_' + h]:.2f}, "
              f"z {params['ez_' + h]:+.2f} +/- {sd['ez_' + h]:.2f} in (z also holds any camera height error)")

    print("\n  Per stop, median tag error after the fit (inches):")
    by_stop = defaultdict(list)
    for o, e in zip(obs, err1):
        by_stop[o['stop']].append(e)
    print("   " + "  ".join(f"{s}:{med(v):.1f}" for s, v in sorted(by_stop.items())))

    print("\n  Paste into CameraMount.java (HEIGHT_IN unchanged):")
    print(f"    public static double FORWARD_IN = {params['F']:.2f};")
    print(f"    public static double LEFT_IN = {params['L']:.2f};")
    print(f"    public static double YAW_DEG = {params['yaw']:.2f};")
    print(f"    public static double PITCH_DEG = {params['pitch']:.2f};")
    print(f"    public static double ROLL_DEG = {params['roll']:.2f};")
    print(f"    public static double DISTANCE_SCALE = {params['scale']:.4f};")


def self_check():
    cam = (0.7, 7.1, 2.9, 91.0, 38.0, 0.6, 1.02)
    for v in ((0.1, -0.2, 1.3), (-0.4, 0.3, 0.9)):
        back = robot_to_camera(cam, camera_to_robot(cam, v))
        assert max(abs(a - b) for a, b in zip(v, back)) < 1e-12, "camera transform round trip failed"


if __name__ == '__main__':
    self_check()
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
