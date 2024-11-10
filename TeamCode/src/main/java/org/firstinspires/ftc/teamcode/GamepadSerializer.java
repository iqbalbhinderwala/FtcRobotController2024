package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.StringBuilder;

public class GamepadSerializer {

    public static String serialize(Gamepad gamepad) {
        StringBuilder sb = new StringBuilder();
        // Serialize button states
        sb.append(gamepad.a ? "1" : "0");
        sb.append(gamepad.b ? "1" : "0");
        sb.append(gamepad.x ? "1" : "0");
        sb.append(gamepad.y ? "1" : "0");

        sb.append(gamepad.dpad_up ? "1" : "0");
        sb.append(gamepad.dpad_down ? "1" : "0");
        sb.append(gamepad.dpad_left ? "1" : "0");
        sb.append(gamepad.dpad_right ? "1" : "0");

        sb.append(gamepad.left_bumper ? "1" : "0");
        sb.append(gamepad.right_bumper ? "1" : "0");

        sb.append(gamepad.left_trigger);
        sb.append(gamepad.right_trigger);

        sb.append(gamepad.guide ? "1" : "0");
        sb.append(gamepad.back ? "1" : "0");
        sb.append(gamepad.start ? "1" : "0");

        // Serialize joystick values
        sb.append(gamepad.left_stick_button ? "1" : "0");
        sb.append(",").append(gamepad.left_stick_x);
        sb.append(",").append(gamepad.left_stick_y);

        sb.append(gamepad.right_stick_button ? "1" : "0");
        sb.append(",").append(gamepad.right_stick_x);
        sb.append(",").append(gamepad.right_stick_y);

        return sb.toString();
    }

    public static Gamepad deserialize(String serializedData) {
        String[] parts = serializedData.split(",");
        int index = 0;

        Gamepad gamepad = new Gamepad();
        // Deserialize button states
        gamepad.a = parts[index++].equals("1");
        gamepad.b = parts[index++].equals("1");
        gamepad.x = parts[index++].equals("1");
        gamepad.y = parts[index++].equals("1");

        gamepad.dpad_up = parts[index++].equals("1");
        gamepad.dpad_down = parts[index++].equals("1");
        gamepad.dpad_left = parts[index++].equals("1");
        gamepad.dpad_right = parts[index++].equals("1");

        gamepad.left_bumper = parts[index++].equals("1");
        gamepad.right_bumper = parts[index++].equals("1");

        gamepad.left_trigger = Float.parseFloat(parts[index++]);
        gamepad.right_trigger = Float.parseFloat(parts[index++]);

        gamepad.guide = parts[index++].equals("1");
        gamepad.back = parts[index++].equals("1");
        gamepad.start = parts[index++].equals("1");

        // Deserialize joystick values
        gamepad.left_stick_button = parts[index++].equals("1");
        gamepad.left_stick_x = Float.parseFloat(parts[index++]);
        gamepad.left_stick_y = Float.parseFloat(parts[index++]);

        gamepad.right_stick_button = parts[index++].equals("1");
        gamepad.right_stick_x = Float.parseFloat(parts[index++]);
        gamepad.right_stick_y = Float.parseFloat(parts[index++]);

        return gamepad;
    }
}
