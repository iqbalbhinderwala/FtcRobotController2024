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

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
@Disabled
public class RobotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  frontRight   = null;
    public DcMotor  frontLeft  = null;
    public DcMotor  backRight     = null;
    public DcMotor  backLeft    = null;

    public DcMotor armLower = null;
    public DcMotor armUpper = null;
    public DcMotor claw = null;

    public DcMotor lift = null;

    public TouchSensor upperSensor = null;
    public int armUpperPosition = 0;
    public int armUpperTargetPosition = 0;
    public int upperEncoderMargin = 100;
    public float upperEncoderMultiplier = 500;
    public boolean upperStop = false;
    public int upperMax = 0;
    public int upperMin = -8500;


    public TouchSensor lowerSensor = null;
    public int armLowerPosition = 0;
    public int armLowerTargetPosition = 0;
    public boolean lowerStop = false;
    public int lowerEncoderMargin = 100;
    public float lowerEncoderMultiplier = 500;
    public int lowerMin = 0;
    public int lowerMax = 12000;


    public TouchSensor clawSensor = null;
    public int clawPosition = 0;
    public int clawTargetPosition = 0;
    public boolean clawStop = false;
    public int clawEncoderMargin = 100;
    public float clawEncoderMultiplier = 500;
    public int clawMin = 0;
    public int clawMax = 11000;

    Servo clawFingers;

    public double MAX_SPEED = 0.8;

    void move()
    {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = gamepad1.left_stick_y * -0.2;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x * 0.2;
        double yaw     =  gamepad1.right_stick_x * 0.2;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if(max > MAX_SPEED)
            max = MAX_SPEED;

        if (max == MAX_SPEED) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        // Send calculated power to wheels
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }


    @Override
    public void runOpMode() {


        // Define and Initialize Motors
        frontRight  = hardwareMap.get(DcMotor.class, "motor 2");
        frontLeft = hardwareMap.get(DcMotor.class, "motor 3");
        backRight    = hardwareMap.get(DcMotor.class, "motor 1");
        backLeft    = hardwareMap.get(DcMotor.class, "motor 4");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        armLower = hardwareMap.get(DcMotor.class, "motor 5");
        armUpper = hardwareMap.get(DcMotor.class, "motor 6");
        claw = hardwareMap.get(DcMotor.class, "motor 7");
        lift = hardwareMap.get(DcMotor.class, "motor 8");

        lowerSensor = hardwareMap.get(TouchSensor.class,"sensor 6");
        upperSensor = hardwareMap.get(TouchSensor.class,"sensor 5");
        clawSensor = hardwareMap.get(TouchSensor.class,"sensor 8");
        clawFingers = hardwareMap.get(Servo.class,"servo 0");

        armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armUpper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad1.back ){
            telemetry.addData("",  "Starting calibration");
            telemetry.update();
            int lowerIncrement = 0;
            int upperIncrement = 0;
            int clawIncrement = 0;
            while(gamepad1.back) {

                if(gamepad1.right_trigger > 0.5 && !gamepad1.right_bumper  && !upperSensor.isPressed()) {
                    upperIncrement -= 100;
                }
                if(gamepad1.right_trigger > 0.5 && gamepad1.right_bumper  && !upperSensor.isPressed()) {
                    upperIncrement += 100;
                }
                if(gamepad1.left_trigger > 0.5 && !gamepad1.left_bumper  && !lowerSensor.isPressed()) {
                    lowerIncrement += 100;
                }
                if(gamepad1.left_trigger > 0.5 && gamepad1.left_bumper  && !lowerSensor.isPressed()) {
                    lowerIncrement -= 100;
                }

                if(gamepad1.dpad_up   && !clawSensor.isPressed()) {
                    clawIncrement += 100;
                }

                if(gamepad1.dpad_down  && !clawSensor.isPressed() && !clawSensor.isPressed()) {
                    clawIncrement -= 100;
                }

                if(upperSensor.isPressed() && lowerSensor.isPressed()){
                    armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    telemetry.addData("",  "Exiting calibration");
                    telemetry.update();

                    break;
                }else {
                    telemetry.addData("",  "Lower %d, Upper %d, LS %b,US %b, CS",lowerIncrement,upperIncrement,lowerSensor.isPressed(),upperSensor.isPressed(),clawSensor.isPressed());
                    telemetry.update();
                    claw.setTargetPosition(clawIncrement );
                    claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    claw.setPower(1.0);
                    armLower.setTargetPosition(lowerIncrement);
                    armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLower.setPower(1.0);
                    armUpper.setTargetPosition(upperIncrement);
                    armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armUpper.setPower(1.0);
                }
                sleep(50);
            }
        }


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot ready 6  Press START.");    //
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            float lowerControl = gamepad1.left_trigger;
            float upperControl = gamepad1.right_trigger;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean lowerOn = false;
            boolean upperOn = false;

            armLowerPosition = armLower.getCurrentPosition();
            armUpperPosition = armUpper.getCurrentPosition();
            clawPosition = claw.getCurrentPosition();
            int upperDelta = abs(armUpperPosition - armUpper.getTargetPosition());
            int lowerDelta = abs(armLowerPosition - armLower.getTargetPosition());
            int clawDelta =  abs(clawPosition - claw.getTargetPosition());
            boolean overrideLimits = gamepad1.x;

            if (rb)
                upperControl = upperControl * upperEncoderMultiplier;
            else
                upperControl = upperControl * -upperEncoderMultiplier;
            armUpperTargetPosition += (int)upperControl;
            if(armUpperTargetPosition <= upperMin && !overrideLimits) armUpperTargetPosition = upperMin;
            if(armUpperTargetPosition >= upperMax && !overrideLimits)  armUpperTargetPosition = upperMax;
            if(upperDelta < upperEncoderMargin || (upperDelta > upperEncoderMargin && !armUpper.isBusy())) {
                armUpper.setTargetPosition(armUpperTargetPosition);
                armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armUpper.setPower(1.0);
                upperOn = true;
            }

            if(lb)
                lowerControl = lowerControl * -lowerEncoderMultiplier;
            else
                lowerControl = lowerControl * lowerEncoderMultiplier;
            armLowerTargetPosition += (int)lowerControl;
            if(armLowerTargetPosition <= lowerMin && !overrideLimits) armLowerTargetPosition = lowerMin;
            if(armLowerTargetPosition >= lowerMax && !overrideLimits)  armLowerTargetPosition = lowerMax;
            if(lowerDelta < lowerEncoderMargin || (lowerDelta > lowerEncoderMargin && !armLower.isBusy())) {
                armLower.setTargetPosition(armLowerTargetPosition);
                armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLower.setPower(1.0);
                lowerOn = true;
            }

            if(gamepad1.dpad_up){
                clawTargetPosition += (int)clawEncoderMultiplier;
                if(clawTargetPosition <= clawMin && !overrideLimits) clawTargetPosition = clawMin;
                if(clawTargetPosition >= clawMax && !overrideLimits)  clawTargetPosition = clawMax;
                if(clawDelta < lowerEncoderMargin || (clawDelta > clawEncoderMargin && !claw.isBusy())) {
                    claw.setTargetPosition(clawTargetPosition);
                    claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    claw.setPower(1.0);
                }
            }
            if(gamepad1.dpad_down) {
                clawTargetPosition -= (int)clawEncoderMultiplier;
                if(clawTargetPosition <= clawMin && !overrideLimits) clawTargetPosition = clawMin;
                if(clawTargetPosition >= clawMax && !overrideLimits)  clawTargetPosition = clawMax;
                if(clawDelta < lowerEncoderMargin || (clawDelta > clawEncoderMargin && !claw.isBusy())) {
                    claw.setTargetPosition(clawTargetPosition);
                    claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    claw.setPower(1.0);
                }
            }
            if(gamepad1.dpad_left){
                clawFingers.setPosition(0.0);
            }
            if(gamepad1.dpad_right){
                clawFingers.setPosition(0.3);
            }

            move();

            // Send telemetry message to signify robot running;
            //telemetry.addData("",  "LT = %.2f, UT = %.2f", lowerControl,upperControl);
            telemetry.addData("",  "Claw fingers %.2f", clawFingers.getPosition());
            telemetry.addData("",  "UACP = %d, LACP = %d", armUpperPosition,armLowerPosition);
            telemetry.addData("",  "UASP = %d, LASP = %d", armUpperTargetPosition,armLowerTargetPosition);
            telemetry.addData("",  "UATP = %d, LATP = %d", armUpper.getTargetPosition(),armLower.getTargetPosition());
            telemetry.addData("",  "Udiff = %d, Ldiff = %d", upperDelta,lowerDelta);
            telemetry.addData("",  "US:%b, LS:%b, CS:%b, LiOv:%b, UOn:%b, LOn:%b", upperSensor.isPressed(),lowerSensor.isPressed(),clawSensor.isPressed(),overrideLimits,upperOn,lowerOn);

            telemetry.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
