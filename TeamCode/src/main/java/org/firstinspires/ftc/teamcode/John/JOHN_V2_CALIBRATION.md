# John v2: calibration checklist

*Started 2026-11-13, after the rebuild for the second competition.*

What changed in the rebuild: rewired hubs, a **gear belt** on the shooter (was a slip belt), the
intakes **raised off the floor** (no more drag), and possibly the camera and pod mounts. Every
calibration below was done on v1 and must be redone or at least checked. Work top to bottom: each
section depends on the ones above it.

Mark `[x]` when done, and write the result and the run number next to it (`r10xx`).

**Logs:** v2 logs go to `/sdcard/FIRST/john_v2_logs/` on the robot, run numbers from **r1001**, and
are archived to `C:\ftc\john_v2_logs\<date>\`. v1 logs stay in `/sdcard/FIRST/john_logs/` and
`C:\ftc\john_logs\` (r0001–r0288). Runs r0289 and later in the old folder were made after the
rebuild: they belong in the v2 archive.

---

## 1. Wiring and directions (`[John] Hardware Check`)

- [x] `John.xml` Expansion Hub ports: intakeA 0, intakeB 1, indexer 2, shooter 3 (2026-11-12)
- [x] Copied to the robot and John re-activated on the Driver Station
- [x] Intake A (left of the shooter) and intake B (right) pull in
- [ ] Indexer: one step turns toward the flywheel, angle +90
- [ ] Flywheel: spins the launching way, RPM positive
- [ ] Drive wheels (on a stand): each wheel alone, rolling toward intake B
  - [ ] frontLeft  - [ ] frontRight  - [ ] backLeft  - [ ] backRight
- [ ] Pinpoint: push toward intake B, X up; toward the shooter side, Y up; turn CCW, heading up
- [ ] Hub IMU: turn CCW, heading up
- [ ] Defender: stows and deploys (recheck 0.2 / 0.8 in `[John] Defender Calibration`)
- [ ] LEDs: colors match; set `StatusLights.LOWER_PORT_IS_RED`
- [ ] Camera: sees the HIVE tags

## 2. Odometry

- [ ] Pod offsets measured on the new build (strafe pod `yPodOffset`, forward pod `xPodOffset`,
  in `Pedro/Constants.java`)
- [ ] Push test over a tape-measured distance (`[John] Odometry Test`): reads within 0.2"
  (v1: 20.092 ticks/mm)
- [ ] Spin test (`[John] Odometry Test`): drift circle under 1" (v1: 0.3–0.8")

## 3. Drive tuning (the intakes no longer drag)

- [ ] `[John] Drive Calibration`: forward / strafe friction push (`forwardKs` / `strafeKs`, v1 0.06 / 0.15)
- [ ] `[John] Heading Calibration`: turn KP and braking (v1 KP 5.2, brake A 0.05, B 0.006)
- [ ] `[John] Hold Test`: holds a pose; moves settle within 0.25", turns within 0.5°
- [ ] Pedro paths: a few drives, including turning while driving

## 4. Shooter

- [ ] New belt ratio: motor turns per flywheel turn = ____ (`Flywheel.GEARING`, v1 1.0)
- [ ] `[John] Flywheel Calibration` on a fresh battery: learned F = ____ (v1 18.76), hand-off, I
- [ ] Check the speed holds: no volley waits for the flywheel (`fw_rpm` vs `fw_target` in a log)
- [ ] Indexer jam handling (`[John] Defender Calibration`, A held, a late ball): the log says
  `backed off X of 90 deg`; does try 1 or 2 free it?
- [ ] Pause between balls: 250 / 200 / 150 ms (`JohnShooter.SHOT_INTERVAL_MS`)

## 5. Camera

- [ ] Mount unchanged? If moved: measure and set `RobotGeometry.CAMERA_*` (height, pitch, offsets)
- [ ] TeleOp INIT flush on a back wall: "X check" within 2" of the wall
- [ ] `[John] Tip Detector Calibration` (if the mount moved): heights within 2" of nominal

## 6. Shooting

- [ ] `[John] Shot Calibration`: coverage starts empty for v2 (it only reads v2 logs). Cover the
  tier 1 spots, then tier 2 and 3
- [ ] Refit `Shooter.RPM_DISTANCES_IN` / `RPM_VALUES` (`C:\ftc\john_v2_logs\shot_table.py`)
- [ ] Defender interference: lowest-out brace position that the balls clear = ____

## 7. Autonomous and TeleOp

- [ ] `[John] Auto` in STEP MODE, then full runs: both volleys, GARDEN, park
- [ ] Park gaps measured (camera wall ____", intake A ____")
- [ ] `[John] TeleOp`: field-centric, A shot, X pre-spin toggle, defender (button still to choose)

## 8. Reliability

- [ ] Shake test (power and the Pinpoint cable, see `JOHN_TODO.md` section 8)
- [ ] One full-length practice match with no power loss or Pinpoint fault
