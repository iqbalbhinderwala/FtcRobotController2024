package org.firstinspires.ftc.teamcode.John.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;

import java.util.Locale;

/**
 * Driving without the Pinpoint: the 4 drive motors directly (mecanum, the names and directions of
 * Pedro's Constants.drivetrainConfig) and the Control Hub IMU ("imu" in John.xml) for the heading.
 * For when the odometry fails (OdometryWatchdog PAUSED / LOST): Pedro's follower is not used then.
 *
 * Call update() once per loop (one IMU read). Heading: field heading = IMU yaw + an offset. While
 * the Pinpoint is good, call alignTo(its heading) every loop, so the IMU carries on from there
 * when it fails. alignTo(a wall heading) re-aligns it against a wall.
 *
 * The hub's mounting (LOGO / USB) must be right for the yaw to be the robot's heading; check the
 * TeleOp's "IMU check" line while the Pinpoint is fine: the offset must stay the same while John
 * turns (if it changes with the turn, LOGO / USB are wrong).
 */
@Config("JohnFallbackDrive")
public class FallbackDrive {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;
    private final String error;
    private double offset;
    private double yaw;
    private double turnRate;
    private boolean modesSet;

    public FallbackDrive(HardwareMap hardwareMap) {
        frontLeft = motor(hardwareMap, Constants.drivetrainConfig.frontLeftName.get(),
                Constants.drivetrainConfig.frontLeftDirection.get());
        frontRight = motor(hardwareMap, Constants.drivetrainConfig.frontRightName.get(),
                Constants.drivetrainConfig.frontRightDirection.get());
        backLeft = motor(hardwareMap, Constants.drivetrainConfig.backLeftName.get(),
                Constants.drivetrainConfig.backLeftDirection.get());
        backRight = motor(hardwareMap, Constants.drivetrainConfig.backRightName.get(),
                Constants.drivetrainConfig.backRightDirection.get());
        IMU found = null;
        String err = "";
        try {
            found = hardwareMap.get(IMU.class, "imu");
            found.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO, USB)));
            found.resetYaw();
        } catch (RuntimeException e) {
            found = null;
            err = "no IMU: " + e.getMessage();
        }
        imu = found;
        error = err;
    }

    private static DcMotor motor(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        DcMotor m = hardwareMap.get(DcMotor.class, name);
        m.setDirection(direction);
        return m;
    }

    public boolean imuOk() {
        return imu != null;
    }

    /** Reads the IMU (yaw and turn rate). Call once per loop. */
    public void update() {
        if (imu != null) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            turnRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        }
    }

    /** Field heading (radians): IMU yaw + the offset (as of the last update()). */
    public double heading() {
        return normalize(yaw + offset);
    }

    /** Turn rate (radians / s, counter-clockwise, as of the last update()). */
    public double turnRate() {
        return turnRate;
    }

    /** Makes heading() read {@code fieldHeading} now (from the Pinpoint, or a wall). */
    public void alignTo(double fieldHeading) {
        offset = normalize(fieldHeading - yaw);
    }

    /** Field heading minus IMU yaw (degrees): steady while turning when the IMU mounting is right. */
    public double offsetDeg() {
        return Math.toDegrees(offset);
    }

    /**
     * Field-centric: (forward, left) from the driver's view, who looks along {@code driverFacing}
     * (field radians); turn counter-clockwise. Powers -1..1.
     */
    public void driveFieldCentric(double forward, double left, double turn, double driverFacing) {
        double a = driverFacing - heading();
        drive(forward * Math.cos(a) - left * Math.sin(a), forward * Math.sin(a) + left * Math.cos(a), turn);
    }

    /** Robot-centric mecanum: x forward, y left, turn counter-clockwise. */
    public void drive(double x, double y, double turn) {
        if (!modesSet) {
            modesSet = true;
            for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        double fl = x - y - turn;
        double bl = x + y - turn;
        double fr = x + y + turn;
        double br = x - y + turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    public void stop() {
        drive(0.0, 0.0, 0.0);
    }

    public String status() {
        return imu == null ? error : String.format(Locale.US, "IMU heading %.1f deg", Math.toDegrees(heading()));
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
