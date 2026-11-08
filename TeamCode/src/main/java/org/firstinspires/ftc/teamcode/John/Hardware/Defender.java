package org.firstinspires.ftc.teamcode.John.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * The defender brace ("defender" in John.xml, Control Hub servo port 0): folds out of the robot to keep
 * opponents from parking in front of the shooter. STOWED_POSITION / DEPLOYED_POSITION come from
 * [John] Defender Calibration.
 *
 * Size rules (R105): expanded, John must fit an 18" x 24" footprint (he is 17" x 17.25"), and the
 * limit must be MECHANICAL (a hard stop), not these servo positions: inspectors check the brace at
 * its full mechanical travel. In the STARTING CONFIGURATION (18" cube, R102) it must be stowed.
 *
 * Nothing moves until stow() / deploy() / setPosition() is called: a servo gets no signal (and holds
 * nothing) until its first position.
 */
@Config("JohnDefender")
public class Defender {

    /**
     * Servo position folded into the robot, close to horizontal (calibrated 2026-11-07: 0.2; the brace
     * is vertical at 0.5). NaN would mean not calibrated: stow() then does nothing.
     */
    public static double STOWED_POSITION = 0.2;
    /** Servo position folded out, close to horizontal (calibrated 2026-11-07: 0.8). */
    public static double DEPLOYED_POSITION = 0.8;
    /** Brace straight up (2026-11-07). */
    public static double UPRIGHT_POSITION = 0.5;
    /** Time the brace takes to swing fully (for deployed() / stowed() checks). */
    public static double MOVE_MS = 600;
    /** Wider pulse range (500-2500 us) for servos that need it to reach their full travel. */
    public static boolean EXTENDED_PWM = false;

    private final Servo servo;
    private final com.qualcomm.robotcore.util.ElapsedTime sinceMove = new com.qualcomm.robotcore.util.ElapsedTime();
    private double position = Double.NaN;
    private Boolean deployed;   // null until the first stow() / deploy()

    public Defender(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "defender");
        applyPwmRange();
    }

    /** Applies EXTENDED_PWM (call again after changing it). */
    public void applyPwmRange() {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(EXTENDED_PWM ? new PwmControl.PwmRange(500, 2500)
                    : PwmControl.PwmRange.defaultRange);
        }
    }

    /** Folds out; does nothing until DEPLOYED_POSITION is calibrated. */
    public void deploy() {
        if (Double.isNaN(DEPLOYED_POSITION)) return;
        move(DEPLOYED_POSITION);
        deployed = true;
    }

    /** Folds in; does nothing until STOWED_POSITION is calibrated. */
    public void stow() {
        if (Double.isNaN(STOWED_POSITION)) return;
        move(STOWED_POSITION);
        deployed = false;
    }

    public static boolean calibrated() {
        return !Double.isNaN(STOWED_POSITION) && !Double.isNaN(DEPLOYED_POSITION);
    }

    /** Any position 0..1 (calibration). */
    public void setPosition(double p) {
        move(p);
        deployed = null;
    }

    private void move(double p) {
        p = Math.max(0.0, Math.min(1.0, p));
        if (p != position) {
            sinceMove.reset();
        }
        position = p;
        servo.setPosition(p);
    }

    /** Last commanded position, NaN before the first. */
    public double position() {
        return position;
    }

    /** Commanded out and had time to get there. */
    public boolean isDeployed() {
        return Boolean.TRUE.equals(deployed) && sinceMove.milliseconds() >= MOVE_MS;
    }

    public boolean isStowed() {
        return Boolean.FALSE.equals(deployed) && sinceMove.milliseconds() >= MOVE_MS;
    }

    /** Port and signal state, for checking the wiring. */
    public String diagnostics() {
        String pwm = servo.getController() instanceof com.qualcomm.robotcore.hardware.ServoControllerEx
                ? String.valueOf(((com.qualcomm.robotcore.hardware.ServoControllerEx) servo.getController())
                        .isServoPwmEnabled(servo.getPortNumber()) ? "PWM on" : "PWM off")
                : String.valueOf(servo.getController().getPwmStatus());
        return servo.getConnectionInfo() + ", " + pwm;
    }

    public String status() {
        String state = deployed == null ? (Double.isNaN(position) ? "not driven" : "manual")
                : deployed ? (isDeployed() ? "DEPLOYED" : "deploying") : (isStowed() ? "stowed" : "stowing");
        return Double.isNaN(position) ? state : String.format(java.util.Locale.US, "%s (%.3f)", state, position);
    }
}
