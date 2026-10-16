package org.firstinspires.ftc.teamcode.Barry.Hardware;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;

/**
 * Dual intake STUB: no hardware yet. It only tracks which side is running and reports it.
 * Replace the bodies of on / off with the real motors later; the Auto keeps using the commands.
 *
 * The robot has one intake on its front and one on its back. With heading 0 on red, the back
 * intake faces red and the front faces blue (and the other way round on blue, which is mirrored).
 */
@Config
public class Intake {

    public enum Side { FRONT, BACK }

    /** Time to keep running after the robot stops, so the last ball gets pulled in. */
    public static double SETTLE_MS = 300;

    private Side running = null;

    public void on(Side side) {
        running = side;
    }

    public void off() {
        running = null;
    }

    public String status() {
        return (running == null ? "off" : running + " running") + " (stub)";
    }

    public Command start(Side side) {
        return instant(() -> on(side));
    }

    public Command stop() {
        return instant(this::off);
    }
}
