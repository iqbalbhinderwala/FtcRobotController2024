package org.firstinspires.ftc.teamcode.John.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** See JohnAutoTest. */
@Autonomous(name = "[John] Auto TEST 15 deg", group = "John", preselectTeleOp = "[John] TeleOp")
public class JohnAutoTest15 extends JohnAutoTest {
    @Override
    protected void apply() {
        newFeatures(15.0, "TEST 15 deg (wall touch, back-off, volley-2 ramp)");
    }
}
