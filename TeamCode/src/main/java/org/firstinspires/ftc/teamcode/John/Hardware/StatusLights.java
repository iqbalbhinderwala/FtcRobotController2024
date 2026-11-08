package org.firstinspires.ftc.teamcode.John.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

/**
 * John's two REV Digital LED Indicators (the part Vex used), on the Control Hub's digital ports: LED 1
 * on 0-1 ("led0", "led1"), LED 2 on 2-3 ("led2", "led3"). Each indicator has a red and a green
 * channel on its two ports; both on is amber. Which port of a pair is red: LOWER_PORT_IS_RED, to be
 * checked with [John] Defender Calibration's LED test.
 */
@Config("JohnStatusLights")
public class StatusLights {

    public enum Color { OFF, RED, GREEN, AMBER }

    /** True if the lower port of each pair (0, 2) lights red, the higher (1, 3) green. */
    public static boolean LOWER_PORT_IS_RED = true;

    private final LED[] channels = new LED[4];

    public StatusLights(HardwareMap hardwareMap) {
        for (int i = 0; i < 4; i++) {
            channels[i] = hardwareMap.get(LED.class, "led" + i);
        }
    }

    /** {@code light} 1 or 2. */
    public void set(int light, Color color) {
        int lower = light == 1 ? 0 : 2;
        boolean red = color == Color.RED || color == Color.AMBER;
        boolean green = color == Color.GREEN || color == Color.AMBER;
        channels[lower].enable(LOWER_PORT_IS_RED ? red : green);
        channels[lower + 1].enable(LOWER_PORT_IS_RED ? green : red);
    }

    /** One raw channel (port 0..3), for testing. */
    public void channel(int port, boolean on) {
        channels[port].enable(on);
    }

    public void off() {
        set(1, Color.OFF);
        set(2, Color.OFF);
    }
}
