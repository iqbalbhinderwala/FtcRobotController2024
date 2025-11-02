/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Vex.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

/* FROM SAMPLE: BasicOmniOpMode_Linear.java
 *
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward                Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="[Vex] Wheels Hardware Test", group="VexTest")
//@Disabled
public class VexWheelsHWTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime lastPress = new ElapsedTime();

    private ElapsedTime deltaTime = new ElapsedTime();
    int leftFrontLastPos = 0;
    int leftBackLastPos = 0;
    int rightFrontLastPos = 0;
    int rightBackLastPos = 0;

    double leftFrontVelocity = 0.0;
    double leftBackVelocity = 0.0;
    double rightFrontVelocity = 0.0;
    double rightBackVelocity = 0.0;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor odometerL = null; // axial left
    private DcMotor odometerR = null; // axial right
    private DcMotor odometerH = null; // lateral (horizontal)

    private final double PRESS_DELAY = 0.25;
    double maxPower = 0.5;
    boolean individualWheelControl = false;
    boolean isForwardDirectionInverted = false;
    boolean enableBrake = false;
    boolean enableEncoders = false;


    private void initMotors() {
        // --- Initialize Drivetrain Motors ---
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "wheel front left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "wheel front right");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "wheel back left");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "wheel back right");

        leftFrontDrive .setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive  .setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive .setDirection(DcMotor.Direction.FORWARD);

        // --- Initialize Odometry Pods ---
        odometerL = hardwareMap.get(DcMotor.class, "wheel front left");
        odometerR = hardwareMap.get(DcMotor.class, "wheel front right");
        odometerH = hardwareMap.get(DcMotor.class, "m3");

        odometerL.setDirection(DcMotor.Direction.REVERSE); // Y +-ve forward
        odometerR.setDirection(DcMotor.Direction.FORWARD); // Y +-ve forward
        odometerH.setDirection(DcMotor.Direction.FORWARD); // X +-ve right
    }

    private void toggleForwardDirection() {
        isForwardDirectionInverted = !isForwardDirectionInverted;

        // Swap leftFrontDrive <-> rightBackDrive
        DcMotor temp = leftFrontDrive;
        leftFrontDrive = rightBackDrive;
        rightBackDrive = temp;

        // Swap leftBackDrive <-> rightFrontDrive
        temp = leftBackDrive;
        leftBackDrive = rightFrontDrive;
        rightFrontDrive = temp;

        // Swap odometerL <-> odometerR
        temp = odometerL;
        odometerL = odometerR;
        odometerR = temp;

        // Invert motor / odometer encoder direction
        for (DcMotor motor : Arrays.asList(odometerL, odometerR, odometerH,
                leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive))
        {
            if (motor.getDirection() == DcMotor.Direction.FORWARD) {
                motor.setDirection(DcMotor.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotor.Direction.FORWARD);
            }
        }
    }

    private void toggleBrakes() {
        enableBrake = !enableBrake;
        for (DcMotor motor : Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive)) {
            motor.setZeroPowerBehavior(enableBrake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    private void toggleEncoders() {
        enableEncoders = !enableEncoders;
        for (DcMotor motor : Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive)) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (enableEncoders) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        for (DcMotor motor : Arrays.asList(odometerL, odometerR, odometerH)) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    @Override
    public void runOpMode() {
        initMotors();
        toggleBrakes();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Invert forward direction
            if (gamepad1.back && lastPress.seconds() >= PRESS_DELAY) {
                lastPress.reset();
                toggleForwardDirection();
            }

            if (gamepad1.left_bumper && lastPress.seconds() >= PRESS_DELAY) {
                lastPress.reset();
                toggleBrakes();
            }

            if (gamepad1.right_bumper && lastPress.seconds() >= PRESS_DELAY) {
                lastPress.reset();
                toggleEncoders();
            }
            telemetry.addData(">","L_STICK (%.1f , %.1f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData(">","R_STICK (%.1f , %.1f)", gamepad1.right_stick_x, gamepad1.right_stick_y);

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value so negate for +ve forward
            double lateral = gamepad1.left_stick_x; // +ve strife right
            double yaw = -gamepad1.right_stick_x;   // +ve turn left ccw
            // NOTE: Assuming robot +X is right and +Y is forward, then +Z-rotation is CCW.

            // D-pad for strafing and straight driving, overrides joystick
            if (gamepad1.dpad_up) {
                axial = 1.0;
                lateral = 0.0;
            } else if (gamepad1.dpad_down) {
                axial = -1.0;
                lateral = 0.0;
            } else if (gamepad1.dpad_left) {
                axial = 0.0;
                lateral = -1.0;
            } else if (gamepad1.dpad_right) {
                axial = 0.0;
                lateral = 1.0;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral - yaw;
            double rightFrontPower = axial - lateral + yaw;
            double leftBackPower = axial - lateral - yaw;
            double rightBackPower = axial + lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Test individual wheels:
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            if (gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b) {
                individualWheelControl = true;
                leftFrontPower  = gamepad1.x ? 1.0 : 0.0;   // X gamepad
                leftBackPower   = gamepad1.a ? 1.0 : 0.0;   // A gamepad
                rightFrontPower = gamepad1.y ? 1.0 : 0.0;   // Y gamepad
                rightBackPower  = gamepad1.b ? 1.0 : 0.0;   // B gamepad
            } else {
                if (individualWheelControl) {
                    individualWheelControl = false;
                    leftFrontPower  = 0.0;
                    leftBackPower   = 0.0;
                    rightFrontPower = 0.0;
                    rightBackPower  = 0.0;
                }
            }

            // Select max speed (don't drive full power)
            if(gamepad1.right_stick_button && lastPress.seconds() >= PRESS_DELAY) {
                lastPress.reset();
                maxPower += 0.1;
                maxPower =  Range.clip(maxPower, 0, 1);
            } else if (gamepad1.left_stick_button && lastPress.seconds() >= PRESS_DELAY) {
                lastPress.reset();
                maxPower -= 0.1;
                maxPower =  Range.clip(maxPower, 0, 1);
            }
            leftFrontPower  *= maxPower;
            rightFrontPower *= maxPower;
            leftBackPower   *= maxPower;
            rightBackPower  *= maxPower;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Calculate velocities
            if (deltaTime.seconds() > 0.5) {
                leftFrontVelocity = (leftFrontDrive.getCurrentPosition() - leftFrontLastPos) / deltaTime.seconds();
                leftBackVelocity = (leftBackDrive.getCurrentPosition() - leftBackLastPos) / deltaTime.seconds();
                rightFrontVelocity = (rightFrontDrive.getCurrentPosition() - rightFrontLastPos) / deltaTime.seconds();
                rightBackVelocity = (rightBackDrive.getCurrentPosition() - rightBackLastPos) / deltaTime.seconds();
                deltaTime.reset();
                leftFrontLastPos = leftFrontDrive.getCurrentPosition();
                leftBackLastPos = leftBackDrive.getCurrentPosition();
                rightFrontLastPos = rightFrontDrive.getCurrentPosition();
                rightBackLastPos = rightBackDrive.getCurrentPosition();
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData(">", "DPad for strafe/straight drive");
            telemetry.addData(">", "Stick Buttons L/R to adjust power");
            telemetry.addData(">", "X/Y: Front Wheels (L/R)");
            telemetry.addData(">", "A/B: Back  Wheels (L/R)");
            telemetry.addData(">", "BACK button: Invert forward direction");
            telemetry.addData(">", "L/R bumpers: Toggle BRAKES / ENCODERS");
            telemetry.addData("IsForwardDirectionInverted", isForwardDirectionInverted);
            telemetry.addData("ZeroPowerBrake", enableBrake);
            telemetry.addData("RUN_USING_ENCODER", enableEncoders);
            telemetry.addData("Power Front Left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Power Back  Left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Max power", maxPower);
            telemetry.addData("Velocity Front Left/Right", "%4.2f, %4.2f /s", leftFrontVelocity, rightFrontVelocity);
            telemetry.addData("Velocity Back  Left/Right", "%4.2f, %4.2f /s", leftBackVelocity, rightBackVelocity);
            telemetry.addData("Encoder Front Left/Right", "%d, %d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
            telemetry.addData("Encoder Back  Left/Right", "%d, %d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.addData("Odometer (L/R , H)", "(%d/%d , %d) CNT",
                    odometerL.getCurrentPosition(), odometerR.getCurrentPosition(), odometerH.getCurrentPosition());
            telemetry.addData("Odometer (L/R , H)", "(%.1f/%.1f , %.1f) INCH",
                    odometerL.getCurrentPosition()*ODOMETER_INCH_PER_COUNT,
                    odometerR.getCurrentPosition()*ODOMETER_INCH_PER_COUNT,
                    odometerH.getCurrentPosition()*ODOMETER_INCH_PER_COUNT);
            telemetry.update();
        }
    }

    // https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/
    private static final double ODOMETER_DIAMETER_MM = 48;
    private static final double ODOMETER_COUNT_PER_REVOLUTION = 2000;
    private static final double ODOMETER_MM_PER_COUNT = (ODOMETER_DIAMETER_MM * Math.PI) / ODOMETER_COUNT_PER_REVOLUTION;
    private static final double ODOMETER_INCH_PER_COUNT = ODOMETER_MM_PER_COUNT / 25.4;
}
