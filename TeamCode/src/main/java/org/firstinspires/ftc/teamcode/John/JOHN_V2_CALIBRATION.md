# John v2: calibration checklist

*Started 2026-11-13, after the rebuild for the second competition.*

What changed in the rebuild: rewired hubs, a **gear belt** on the shooter (was a slip belt), the
intakes **raised off the floor** (no more drag), and possibly the camera and pod mounts. Every
calibration below was done on v1 and must be redone or at least checked. Work top to bottom: each
section depends on the ones above it. Each section title names the OpMode(s) to run.

Mark `[x]` when done, and write the result and the run number next to it (`r10xx`).

**Logs:** v2 logs go to `/sdcard/FIRST/john_v2_logs/` on the robot, run numbers from **r1001**, and
are archived to `C:\ftc\john_v2_logs\<date>\`. v1 logs stay in `/sdcard/FIRST/john_logs/` and
`C:\ftc\john_logs\` (r0001–r0288). Runs r0289 and later in the old folder were made after the
rebuild: they belong in the v2 archive.

---

## 1. Wiring and directions: `[John] Hardware Check`

- [x] `John.xml` Expansion Hub ports: intakeA 0, intakeB 1, indexer 2, shooter 3 (2026-11-12)
- [x] Copied to the robot and John re-activated on the Driver Station
- [x] Intake A (left of the shooter) and intake B (right) pull in
- [ ] Indexer: one step turns toward the flywheel, angle +90
- [ ] Flywheel: spins the launching way, RPM positive
- [ ] Drive wheels (on a stand): each wheel alone, rolling toward intake B
  - [ ] frontLeft  - [ ] frontRight  - [ ] backLeft  - [ ] backRight
- [ ] Pinpoint: push toward intake B, X up; toward the shooter side, Y up; turn CCW, heading up
- [ ] Hub IMU: turn CCW, heading up
- [ ] Camera: sees the HIVE tags

## 2. Defender brace and LEDs: `[John] Defender Calibration`

- [ ] Brace positions still right after the rebuild: stowed ____ (v1 0.2), deployed ____ (v1 0.8),
  no servo buzz at either end (`Defender.STOWED_POSITION` / `DEPLOYED_POSITION`)
- [ ] Brace size at full mechanical travel: John fits an 18" x 24" footprint (rule R105; a hard stop,
  not the servo, must keep it there)
- [ ] LED test (Back switches): color of each port. Port 0 ____, 1 ____, 2 ____, 3 ____;
  set `StatusLights.LOWER_PORT_IS_RED`

## 3. Odometry: `[John] Odometry Test`

- [ ] Pod offsets measured on the new build (strafe pod `yPodOffset`, forward pod `xPodOffset`,
  in `Pedro/Constants.java`)
- [ ] Push test over a tape-measured distance: reads within 0.2" (v1: 20.092 ticks/mm)
- [ ] Spin test: drift circle under 1" (v1: 0.3–0.8")

## 4. Drive tuning: `[John] Drive Calibration`, `[John] Heading Calibration`, `[John] Hold Test`

The intakes no longer drag, so friction and turning changed.

- [ ] Drive Calibration: forward / strafe friction push, forwardKs ____ / strafeKs ____ (v1 0.06 / 0.15)
- [ ] Heading Calibration: turn KP ____, brake A ____, B ____ (v1 5.2 / 0.05 / 0.006)
- [ ] Hold Test: holds a pose; moves settle within 0.25", turns within 0.5°
- [ ] A few Pedro paths, including turning while driving

## 5. Driver sticks: `[John] TeleOp`

Tune live on the Dashboard (`JohnTeleOp`), then copy the values into `JohnTeleOp.java`.

- [ ] Friction floors from section 4: `DRIVE_MIN_FORWARD` ____ (v1 0.06), `DRIVE_MIN_STRAFE` ____
  (v1 0.15), `TURN_MIN` ____ (v1 0.05). A light touch should creep, not lurch and not do nothing
- [ ] Response curve: `DRIVE_EXPO` ____ (0.6), `TURN_EXPO` ____ (0.7). Higher = gentler at low
  speed; 0 = linear
- [ ] Top speed: `DRIVE_SCALE` ____ (0.8), `TURN_SCALE` ____ (0.6)
- [ ] Drivers happy: fine positioning at the shooting spot, full speed across the field

## 6. Shooter: `[John] Flywheel Calibration`, `[John] Defender Calibration`

- [ ] New belt ratio: motor turns per flywheel turn = ____ (`Flywheel.GEARING`, v1 1.0)
- [ ] Flywheel Calibration on a fresh battery: learned F = ____ (v1 18.76), hand-off, I
- [ ] The speed holds: no volley waits for the flywheel (`fw_rpm` vs `fw_target` in an Auto log)
- [ ] Indexer jams (Defender Calibration, A held, drop a late ball): the log says
  `backed off X of 90 deg`; freed on try ____ (1 / 2 / STUCK)

## 7. Camera: `[John] TeleOp` (INIT), `[John] Tip Detector Calibration`

- [ ] Mount unchanged? If moved: measure and set `RobotGeometry.CAMERA_*` (height, pitch, offsets)
- [ ] TeleOp INIT flush on a back wall: "X check" within 2" of the wall
- [ ] Tip Detector Calibration (if the mount moved): heights within 2" of nominal

## 8. Shooting: `[John] Shot Calibration`, `[John] Defender Calibration`

- [ ] Shot Calibration: coverage starts empty for v2 (it only reads v2 logs). Cover the tier 1
  spots, then tier 2 and 3
- [ ] Refit `Shooter.RPM_DISTANCES_IN` / `RPM_VALUES` (`C:\ftc\john_v2_logs\shot_table.py`)
- [ ] Pause between balls (Shot Calibration at one spot): 250 / 200 / 150 ms
  (`JohnShooter.SHOT_INTERVAL_MS`), best ____
- [ ] Defender interference (Defender Calibration, A held at several brace angles): lowest-out
  position the balls clear = ____

## 9. Match programs: `[John] Game Setup`, `[John] Auto`, `[John] TeleOp`

- [ ] Auto in STEP MODE, then full runs: both volleys, GARDEN, park
- [ ] Park gaps measured (camera wall ____", intake A ____")
- [ ] TeleOp: field-centric, A shot, X pre-spin toggle, defender (button still to choose)

## 10. Reliability: any match program

- [ ] Shake test (power and the Pinpoint cable, see `JOHN_TODO.md` section 8)
- [ ] One full-length practice match with no power loss or Pinpoint fault
