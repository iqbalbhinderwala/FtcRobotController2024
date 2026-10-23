package org.firstinspires.ftc.teamcode.John.Hardware;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;

/**
 * Shooter STUB: no hardware yet. It only waits for realistic times and reports what it would do, so
 * the autonomous can be built and timed now. Replace the bodies of spinUp / fire / stop with the
 * real motors and the gate / indexer later; the autonomous keeps using shoot(n).
 *
 * The shooter fires out of John's shooter side (RobotGeometry.SHOOTER_YAW_DEG); balls wait on the
 * ramp behind the gate / indexer.
 */
@Config("JohnShooter")
public class Shooter {

    /**
     * Flywheel speed by start tile (shooting at the up CELL from the audience wall), measured
     * 2026-10-23 with [John] Mechanism Test (encoder speed control).
     */
    public static double RPM_FROM_C1 = 2200;
    public static double RPM_FROM_B1 = 2300;
    public static double RPM_FROM_A1 = 2400;
    /** 4 balls, plus 2 extra indexer steps in case a ball comes late. */
    public static int INDEXER_STEPS_PER_VOLLEY = 6;

    /** Time to reach launch speed. */
    public static double SPIN_UP_MS = 800;
    /** Time between two balls (200-250 ms worked in the Mechanism Test; 150 ms missed 1 in 4). */
    public static double SHOT_INTERVAL_MS = 250;

    private boolean spinning = false;
    private int shotsFired = 0;
    private String status = "idle";

    public void spinUp() {
        spinning = true;
        status = "spinning up";
    }

    public void fire() {
        shotsFired++;
        status = "fired ball " + shotsFired;
    }

    public void stop() {
        spinning = false;
        status = "stopped";
    }

    public boolean spinning() {
        return spinning;
    }

    public int shotsFired() {
        return shotsFired;
    }

    public String status() {
        return status + " (stub)";
    }

    /** Spins up, fires {@code count} balls, then stops. */
    public Command shoot(int count) {
        return sequential(
                instant(this::spinUp),
                waitMs(SPIN_UP_MS),
                repeat(sequential(instant(this::fire), waitMs(SHOT_INTERVAL_MS)), count),
                instant(this::stop));
    }
}
