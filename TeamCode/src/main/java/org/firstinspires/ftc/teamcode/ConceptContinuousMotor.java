package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode runs a single motor at a constant speed set by the user.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "<motor_name>" as is found on a Robot.
 *
 * SPEED_INCREMENT sets how much to increase/decrease the power each button press.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: Continuous Motor", group = "Concept")
public class ConceptContinuousMotor extends LinearOpMode {

    static final double SPEED_INCREMENT = 0.1;     // amount to change motor speed each button press
    static final int    CYCLE_MS        = 50;      // period of each cycle

    // Define class members
    DcMotor motor;
    double  power = 0; // Initial speed
    double  maxPower = 1.0;
    boolean forwardDirection = true;

    ElapsedTime lastClick = new ElapsedTime();
    double BUTTON_DELAY_SEC = 0.25;

    @Override
    public void runOpMode() {

        // Connect to motor
        // Change the text in quotes to match any motor name on your robot.
        motor = hardwareMap.get(DcMotor.class, "motor");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motor.");
        telemetry.addData(">", "Use dpad up/down to adjust max speed.");
        telemetry.addData(">", "Use back to toggle direction");
        telemetry.addData(">", "Press x to activate motor.");
        telemetry.update();
        waitForStart();

        // Run motor at constant speed until stop pressed.
        while(opModeIsActive()) {

            // Invert forward direction
            if (gamepad1.back && lastClick.seconds() >= BUTTON_DELAY_SEC) {
                lastClick.reset();
                forwardDirection = !forwardDirection;
            }

            // Select max speed
            if(gamepad1.dpad_up && lastClick.seconds() >= BUTTON_DELAY_SEC) {
                lastClick.reset();
                maxPower += SPEED_INCREMENT;
            } else if (gamepad1.dpad_down && lastClick.seconds() >= BUTTON_DELAY_SEC) {
                lastClick.reset();
                maxPower -= SPEED_INCREMENT;
            }

            // Clip the power values so they only range from 0 to 1.0.
            maxPower =  Range.clip(maxPower, 0, 1);

            // Set power if x is pressed
            if (gamepad1.x) {
                power = maxPower * (forwardDirection ? 1.0 : -1.0);
            } else {
                power = 0.0;
            }

            // Display the current value
            telemetry.addData("Max Speed", "%5.2f", maxPower);
            telemetry.addData("Direction", forwardDirection ? "Forward" : "Reverse");
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "DPad Up/Down to adjust max speed.");
            telemetry.addData(">", "Back to toggle direction");
            telemetry.addData(">", "Press X to activate motor.");
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            motor.setPower(power);
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        motor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
