package org.firstinspires.ftc.teamcode.John.Hardware;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * John's two intakes ("intakeA" / "intakeB" in John.xml, Expansion Hub ports 1 / 0, goBILDA 5204),
 * one at each axial end, collecting toward the middle (RobotGeometry): intake A on the shooter's
 * left (Pedro's back end), intake B on its right (Pedro's front end).
 *
 * In the Mechanism Test both always ran together at full power, so on(side) runs both too
 * (BOTH_TOGETHER); A_REVERSED / B_REVERSED flip which way "in" is for each.
 */
@Config("JohnIntake")
public class Intake {

    public enum Side { A, B }

    public static double POWER = 1.0;
    /**
     * Intake A runs the other way round. (2026-10-24: the A / B cables had been swapped, so this was
     * found as "B reversed"; with intakeA on the left intake and intakeB on the right again, the
     * reversal moved with the motor.)
     */
    public static boolean A_REVERSED = true;
    public static boolean B_REVERSED = false;
    /** on(side) runs both intakes (as tested), not just the leading one. */
    public static boolean BOTH_TOGETHER = true;
    /** Time to keep running after the robot stops, so the last ball gets pulled in. */
    public static double SETTLE_MS = 300;

    private final DcMotorEx intakeA;
    private final DcMotorEx intakeB;
    private String status = "off";

    public Intake(HardwareMap hardwareMap) {
        intakeA = hardwareMap.get(DcMotorEx.class, "intakeA");
        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        for (DcMotorEx m : new DcMotorEx[]{intakeA, intakeB}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /** Both intakes at this power (+ in, - out). */
    public void set(double power) {
        apply(power, power);
        status = power > 0 ? "in" : power < 0 ? "reverse" : "off";
    }

    /** Collecting with this side leading (both, with BOTH_TOGETHER). */
    public void on(Side side) {
        boolean a = BOTH_TOGETHER || side == Side.A;
        boolean b = BOTH_TOGETHER || side == Side.B;
        apply(a ? POWER : 0.0, b ? POWER : 0.0);
        status = BOTH_TOGETHER ? "in (both, " + side + " leading)" : "intake " + side + " in";
    }

    /** Only this side's intake at this power (+ in, - out); the other one stops. */
    public void only(Side side, double power) {
        apply(side == Side.A ? power : 0.0, side == Side.B ? power : 0.0);
        status = power == 0 ? "off" : "intake " + side + (power > 0 ? " in" : " reverse");
    }

    public void off() {
        apply(0.0, 0.0);
        status = "off";
    }

    private void apply(double a, double b) {
        intakeA.setDirection(A_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intakeB.setDirection(B_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intakeA.setPower(a);
        intakeB.setPower(b);
    }

    public String status() {
        return status;
    }

    public Command start(Side side) {
        return instant(() -> on(side));
    }

    public Command stop() {
        return instant(this::off);
    }
}
