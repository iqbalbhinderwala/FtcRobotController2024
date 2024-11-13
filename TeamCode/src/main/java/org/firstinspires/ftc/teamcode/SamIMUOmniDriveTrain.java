/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/* FROM SAMPLES RobotAutoDriveToAprilTagOmni.java and RobotAutoDriveByGyro_Linear.java
 *
 */

public class SamIMUOmniDriveTrain
{
    /* Reference to the parent OpMode. */
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.

    boolean isForwardDirectionInverted = false;

    /* Declare Component members. */
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private IMU     imu              = null;   // Control/Expansion Hub IMU

    private double  headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading   = 0;

    private double leftFrontPower  = 0;
    private double rightFrontPower = 0;
    private double leftBackPower   = 0;
    private double rightBackPower  = 0;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double HEADING_THRESHOLD = 1.0 ;   // How close must the heading get to the target before moving to next step.
                                                    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

    static final double MIN_TURN_SPEED = 0.2;

    // Constructor
    public SamIMUOmniDriveTrain(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    private void initMotors() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = opMode.hardwareMap.get(DcMotor.class, "motor 1");
        leftBackDrive   = opMode.hardwareMap.get(DcMotor.class, "motor 2");
        rightFrontDrive = opMode.hardwareMap.get(DcMotor.class, "motor 4");
        rightBackDrive  = opMode.hardwareMap.get(DcMotor.class, "motor 3");

        leftFrontDrive .setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive  .setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive .setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Disable the encoders.
        // TurnToHeading() relies on encoders being disabled.
        // NOTE: RUN_USING_ENCODER positions are always ZERO anyway?!
        leftFrontDrive .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initIMU() {
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Reset the IMU heading.
        imu.resetYaw();
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        leftFrontPower    =  x -y -yaw;     // save this value as a class member so it can be used by telemetry.
        rightFrontPower   =  x +y +yaw;     // save this value as a class member so it can be used by telemetry.
        leftBackPower     =  x +y -yaw;     // save this value as a class member so it can be used by telemetry.
        rightBackPower    =  x -y +yaw;     // save this value as a class member so it can be used by telemetry.

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }


    // Adjust these numbers to suit your robot.
    final double TURN_GAIN   =  0.015 ;    //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading, double timeout) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Run getSteeringCorrection() once to calculate the current heading error
        getSteeringCorrection(heading, TURN_GAIN);

        // Account for overshooting
        double overshoot = 0.10 * Math.abs(headingError);  // 0.10 tuned/tested only for 90 degree turns
        double headingThresholdWithOvershoot = Math.max(HEADING_THRESHOLD, overshoot);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && (Math.abs(headingError) > headingThresholdWithOvershoot)) {
            if( timer.seconds() > timeout){
                opMode.telemetry.addLine("TurnToHeading::Timeout");
                break;
            }
            // Determine required steering to keep on heading
            double turnSpeed = getSteeringCorrection(heading, TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = clampMagnitude(turnSpeed, MIN_TURN_SPEED, maxTurnSpeed); // MIN_TURN_SPEED to prevent stalling

            // Pivot in place by applying the turning correction
            moveRobot(0, 0, turnSpeed);

            headingError = heading - getHeading();

//            opMode.telemetry.addData(">", "yaw=%.2f  err=%.2f turnPwr=%.2f", getHeading(), headingError, turnSpeed);
        }

        // Stop all motion
        moveRobot(0, 0, 0);

        opMode.telemetry.addData(">", "%.1fsec  target=%.2f yaw=%.2f  err=%.2f",
                timer.seconds(), targetHeading, getHeading(), headingError);
//        opMode.telemetry.addLine("Done. Press BACK to continue.");
//        opMode.telemetry.update();
//        while(opMode.opModeIsActive()&&!opMode.gamepad1.back){opMode.sleep(100);}
    }

    private static double clampMagnitude(double value, double min, double max) {
        double magnitude = Math.min(Math.max(Math.abs(value), min), max);
        return Math.copySign(magnitude, value);
    }

    /**
     * Initialize
     */
    public void init() {
        initMotors();
        initIMU();
    }

    /**
     * Stop all motion
     */
    public void stopMotors() {
        // Stop all motion
        moveRobot(0, 0, 0);
    }

    /**
     * Read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset current Robot heading to read zero.
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    public void cruise(double speed, double heading) {
        double theta = Math.toRadians(getHeading() - heading);
        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);
        //https://www.chiefdelphi.com/uploads/default/original/3X/0/7/078afb4a794d5ed568b03a7d766cc1df36358bd9.pdf
        double axial = speed;
        double lateral = 0;
        double forward = axial * cosTheta - lateral * sinTheta;
        double right   = axial * sinTheta + lateral * cosTheta;
        moveRobot(forward, -right, 0);
    }

    public void driveStraight(double power, double heading, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(opMode.opModeIsActive() && timer.seconds() < seconds) {
            cruise(power, heading);
        }
        stopMotors();
    }

}
