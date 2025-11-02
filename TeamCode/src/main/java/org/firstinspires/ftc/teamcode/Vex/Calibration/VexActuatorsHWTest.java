/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="[Vex] Actuators Hardware Test", group="VexTest")
//@Disabled
public class VexActuatorsHWTest extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor intakeMotor = null;
    private DcMotor topShooterMotor = null; // top
    private DcMotor bottomShooterMotor = null; // bottom
    private Servo gateAServo = null;
    private Servo gateBServo = null;

    private void initJointsHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        intakeMotor = hardwareMap.get(DcMotor.class, "m2");
        topShooterMotor = hardwareMap.get(DcMotor.class, "m0");
        bottomShooterMotor = hardwareMap.get(DcMotor.class, "m1");

        gateAServo = hardwareMap.get(Servo.class, "gate a");
        gateBServo = hardwareMap.get(Servo.class, "gate b");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        topShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomShooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {

        initJointsHardware();

        gateAServo.setPosition(0);
        gateBServo.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        for (DcMotor motor : new DcMotor[] {intakeMotor, topShooterMotor, bottomShooterMotor}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        ElapsedTime lastPress = new ElapsedTime();
        final double BUTTON_DELAY = 0.25;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double intakePower   = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative stick_y value
            double shooterPower  = -gamepad1.right_stick_y; // Note: pushing stick forward gives negative stick_y value

            intakeMotor.setPower(intakePower);
            topShooterMotor.setPower(gamepad1.right_stick_button ? 0 : shooterPower);
            bottomShooterMotor.setPower(gamepad1.right_stick_button ? shooterPower : 0);

            double SERVO_INCREMENT = 0.05;
            // GATE A calibration
            if (gamepad1.dpad_up && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                gateAServo.setPosition(Math.min(1.0, gateAServo.getPosition() + SERVO_INCREMENT));
            }
            if (gamepad1.dpad_down && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                gateAServo.setPosition(Math.max(0.0, gateAServo.getPosition() - SERVO_INCREMENT));
            }

            // GATE B calibration
            if (gamepad1.dpad_right && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                gateBServo.setPosition(Math.min(1.0, gateBServo.getPosition() + SERVO_INCREMENT));
            }
            if (gamepad1.dpad_left && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                gateBServo.setPosition(Math.max(0.0, gateBServo.getPosition() - SERVO_INCREMENT));
            }

            telemetry.addData(">", "Left Stick Y: Control Intake");
            telemetry.addData(">", "Right Stick Y: Control Shooters");
            telemetry.addData(">", "DPad U/D: Adjust Gate A");
            telemetry.addData(">", "DPad L/R: Adjust Gate B");
            telemetry.addData("", "--------------------------------");
            telemetry.addData("INTAKE", "Pwr=%.1f", intakeMotor.getPower());
            telemetry.addData("SHOOTER", "Pwr=%.1f", topShooterMotor.getPower());
            telemetry.addData("GATE A", "Pos=%.2f", gateAServo.getPosition());
            telemetry.addData("GATE B", "Pos=%.2f", gateBServo.getPosition());
            telemetry.update();
        }
    }
}
