package org.firstinspires.ftc.teamcode.John.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** See JohnAutoTest. */
@Autonomous(name = "[John] Auto TEST square", group = "John", preselectTeleOp = "[John] TeleOp")
public class JohnAutoTestSquare extends JohnAutoTest {
    @Override
    protected void apply() {
        newFeatures(0.0, "TEST square (0 deg, wall touch, back-off, volley-2 ramp)");
    }
}
