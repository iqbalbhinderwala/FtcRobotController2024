package org.firstinspires.ftc.teamcode.Barry.Hardware;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;

/**
 * Shooter STUB: no hardware yet. It only waits for realistic times and reports what it would do,
 * so the Auto can be built and timed now. Replace the bodies of spinUp / fire / stop with the real
 * motors later; the Auto keeps using shoot(n).
 *
 * The shooter fires along the camera axis (out of the robot's left side), so aiming the camera
 * at a cluster also aims the shooter.
 */
@Config
public class Shooter {

    /** Time to reach launch speed. */
    public static double SPIN_UP_MS = 800;
    /** Time between two balls. */
    public static double SHOT_INTERVAL_MS = 350;

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
