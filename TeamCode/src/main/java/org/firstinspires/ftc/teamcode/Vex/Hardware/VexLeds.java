package org.firstinspires.ftc.teamcode.Vex.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;

public class VexLeds {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.

    /* Declare Component members. */
    private LED led0 = null;
    private LED led1 = null;
    private LED led2 = null;
    private LED led3 = null;

    public VexLeds(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // Initialize all hardware components.
    public void init() {
        led0 = opMode.hardwareMap.get(LED.class, "led0");
        led1 = opMode.hardwareMap.get(LED.class, "led1");
        led2 = opMode.hardwareMap.get(LED.class, "led2");
        led3 = opMode.hardwareMap.get(LED.class, "led3");
    }

    // Start all hardware components.
    public void start() {
    }

    public LED getLed(int index) {
        switch (index) {
            case 0: return led0;
            case 1: return led1;
            case 2: return led2;
            case 3: return led3;
            default: return null;
        }
    }
}
