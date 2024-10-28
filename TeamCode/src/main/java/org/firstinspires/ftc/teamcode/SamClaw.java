package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class SamClaw {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.

    /* Declare Component members. */

    // Define class members
    Servo servo;

    public SamClaw(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    public void init() {
        servo = opMode.hardwareMap.get(Servo.class, "servo 0");
    }

    public void open() {
        servo.setPosition(OPEN_POS);
    }

    public void closed() {
        servo.setPosition(CLOSED_POS);
    }

    public void toggle() {
        if (isCloserToFirst(
                OPEN_POS,
                CLOSED_POS,
                servo.getPosition())) {
            servo.setPosition(CLOSED_POS);
        } else {
            servo.setPosition(OPEN_POS);
        }
    }

    private static boolean isCloserToFirst(double first, double second, double target) {
        // Calculate the absolute difference between num1, num2 and target
        double diff1 = Math.abs( first - target);
        double diff2 = Math.abs(second - target);
        return diff1 <= diff2;
    }

    final double OPEN_POS   = 0.15;
    final double CLOSED_POS = 0.0;
}
