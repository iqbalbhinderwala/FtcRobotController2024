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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="[Sam] Joints Hardware Test", group="SamTest")
//@Disabled
public class SamJointsHWTest extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor baseMotor = null;
    private DcMotor armMotor = null;
    private DcMotor wristMotor = null;

    private TouchSensor baseSensor = null;
    private TouchSensor armSensor = null;
    private TouchSensor wristSensor = null;

    boolean isBaseCalibrated = false;
    boolean isArmCalibrated = false;
    boolean isWristCalibrated = false;

    private Servo claw = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        baseMotor = hardwareMap.get(DcMotor.class, "motor 5");
        armMotor = hardwareMap.get(DcMotor.class, "motor 6");
        wristMotor = hardwareMap.get(DcMotor.class, "motor 7");

        // +ve raise up; -ve lower towards ground
        baseMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        baseSensor = hardwareMap.get(TouchSensor.class, "sensor 6");
        armSensor = hardwareMap.get(TouchSensor.class, "sensor 5");
        wristSensor = hardwareMap.get(TouchSensor.class, "sensor 8");

        claw = hardwareMap.get(Servo.class,"servo 0");

        telemetry.addData("Sensor state", "Base: %b  Arm: %b  Wrist: %b", baseSensor.isPressed(), armSensor.isPressed(), wristSensor.isPressed());
        telemetry.addData("Current base position", baseMotor.getCurrentPosition());
        telemetry.addData("Current arm position", armMotor.getCurrentPosition());
        telemetry.addData("Current wrist position", wristMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        baseMotor .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        baseMotor .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // run until the end of the match (driver presses STOP)

        ElapsedTime lastPress = new ElapsedTime();
        final double BUTTON_DELAY = 0.1;

        while (opModeIsActive()) {
            double basePower   = -gamepad1.left_stick_y  * 0.5; // Note: pushing stick forward gives negative stick_y value
            double armPower    = -gamepad1.right_stick_y * 0.5; // Note: pushing stick forward gives negative stick_y value
            double wristPower  = (-gamepad1.left_trigger + gamepad1.right_trigger) * 0.5; // TODO: CALIBRATE MAX POWER FOR BASE

            baseMotor.setPower(basePower);
            armMotor.setPower(armPower);
            wristMotor.setPower(wristPower);

            // RESET ENCODERS WHOSE SENSOR IS TRUE
            if (gamepad1.y) {
                if (baseSensor.isPressed())
                {
                    isBaseCalibrated = true;
                    baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (armSensor.isPressed())
                {
                    isArmCalibrated = true;
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (wristSensor.isPressed())
                {
                    isWristCalibrated = true;
                    wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            if (gamepad1.dpad_up && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                claw.setPosition(claw.getPosition()+0.02);
            }
            if (gamepad1.dpad_down && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                claw.setPosition(claw.getPosition()-0.02);
            }

            telemetry.addData(">", "Y: Try reset encoders");
            telemetry.addData(">", "    DPad Up/Down: Adjust Claw");

            telemetry.addData("BASE ", "Pos=%d  Pwr=%.1f\tSense=%.1f", baseMotor.getCurrentPosition(),  baseMotor.getPower(),  baseSensor.getValue());
            telemetry.addData("ARM  ", "Pos=%d  Pwr=%.1f\tSense=%.1f", armMotor.getCurrentPosition(),   armMotor.getPower(),   armSensor.getValue());
            telemetry.addData("WRIST", "Pos=%d  Pwr=%.1f\tSense=%.1f", wristMotor.getCurrentPosition(), wristMotor.getPower(), wristSensor.getValue());
            telemetry.addData("CLAW ", "Pos=%.2f", claw.getPosition());
            telemetry.addData("Sensor (Base)",  baseSensor.isPressed());
            telemetry.addData("Sensor (Arm)",   armSensor.isPressed());
            telemetry.addData("Sensor (Wrist)", wristSensor.isPressed());
            telemetry.addData("Base Calibrated",    isBaseCalibrated);
            telemetry.addData("Arm Calibrated",     isArmCalibrated);
            telemetry.addData("Wrist Calibrated",   isWristCalibrated);
            telemetry.update();
        }
    }
}
