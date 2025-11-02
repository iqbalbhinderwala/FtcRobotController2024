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

package org.firstinspires.ftc.teamcode.Vex.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vex.Hardware.VexActuators;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;

@TeleOp(name="[Vex] Actuators Control Test", group="VexTest")
public class VexActuatorsControlTest extends LinearOpMode {

    private VexActuators actuators = new VexActuators(this);
    private VexOdometryDriveTrain driveTrain;

    @Override
    public void runOpMode() {

        actuators.init(hardwareMap);
        driveTrain = new VexOdometryDriveTrain(this);
        driveTrain.init();
        driveTrain.setPose(0,0,0);


        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Driver orientation is set to face the +Y-Axis (0 deg).");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        ElapsedTime lastPress = new ElapsedTime();
        final double BUTTON_DELAY = 0.25;
        double intakePower = 1.0;
        double shooterPower = 0.5;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // --- READ JOYSTICK INPUTS ---

            // Get raw joystick values for movement and turning.
            double forwardInput = -gamepad1.left_stick_y; // Positive is "forward"
            double strafeInput  =  gamepad1.left_stick_x;  // Positive is "right"
            double turnInput    = -gamepad1.right_stick_x; // Positive is counter-clockwise

            // --- CALL DRIVETRAIN METHOD ---

            // Get the robot's current heading from the IMU.
            double robotHeading = driveTrain.getHeading();

            // Define the driver's fixed orientation. 0 degrees means facing along the +Y axis.
            double humanDirection = 0.0;

            // Call the centralized driving function within the drivetrain class,
            // passing all necessary inputs. The drivetrain now handles all calculations.
            driveTrain.moveHumanCentric(forwardInput, strafeInput, turnInput, robotHeading, humanDirection);

            // --- Adjust Powers ---
            double POWER_INCREMENT = 0.05;
            if (gamepad1.dpad_up && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                shooterPower = Math.min(1.0, shooterPower + POWER_INCREMENT);
            }
            if (gamepad1.dpad_down && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                shooterPower = Math.max(0.0, shooterPower - POWER_INCREMENT);
            }
            if (gamepad1.dpad_right && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                intakePower = Math.min(1.0, intakePower + POWER_INCREMENT);
            }
            if (gamepad1.dpad_left && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                intakePower = Math.max(0.0, intakePower - POWER_INCREMENT);
            }

            // --- Shooting Sequence (Y Button) ---
            if (gamepad1.y) {
                actuators.shootSequence(shooterPower); // blocking
            } else { // Not in shooting sequence

                // --- Manual Shooter (X Button) ---
                if (gamepad1.x) {
                    actuators.setShooterPower(shooterPower);
                } else {
                    actuators.setShooterPower(0);
                }

                // --- Intake (A/B Buttons) ---
                if (gamepad1.a) {
                    actuators.setIntakePower(intakePower);
                    // When intake is on, gate A is closed, and gateB is open
                    actuators.closeGateA();
                    actuators.openGateB();
                } else if (gamepad1.b) {
                    actuators.setIntakePower(-intakePower);
                } else {
                    actuators.setIntakePower(0);
                }

                // --- Manual Gate Control (if not intaking) ---
                if (!gamepad1.a) { // only allow manual if intake is off
                    // LEFT BUMPER to CLOSE, TRIGGER to OPEN
                    if (gamepad1.left_bumper) {
                        actuators.closeGateA();
                    } else if (gamepad1.left_trigger > 0.5) {
                        actuators.openGateA();
                    }

                    // RIGHT BUMPER to CLOSE, TRIGGER to OPEN
                    if (gamepad1.right_bumper) {
                        actuators.closeGateB();
                    } else if (gamepad1.right_trigger > 0.5) {
                        actuators.openGateB();
                    }
                }
            }

            driveTrain.update();
            telemetry.addData("--- Driving ---", "");
            telemetry.addData("Left Stick Y (Fwd)", "%.2f", forwardInput);
            telemetry.addData("Left Stick X (Str)", "%.2f", strafeInput);
            telemetry.addData("Right Stick X (Trn)", "%.2f", turnInput);
            telemetry.addData("Heading (Deg)", "%.1f", driveTrain.getHeading());
            telemetry.addData("Pose", driveTrain.getPose().toString());

            telemetry.addData("--- Actuators ---", "");
            telemetry.addData(">", "A: Intake, B: Reverse, Y: Auto-Shoot, X: Manual-Shoot");
            telemetry.addData(">", "DPad U/D: Shooter pwr, L/R: Intake pwr");
            telemetry.addData(">", "L/R Bumper/Trigger: Manual Gate Control");
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.addData("Gate A Pos", "%.2f", actuators.getGateAPosition());
            telemetry.addData("Gate B Pos", "%.2f", actuators.getGateBPosition());
            telemetry.update();
        }

        driveTrain.stopMotors();
    }
}
