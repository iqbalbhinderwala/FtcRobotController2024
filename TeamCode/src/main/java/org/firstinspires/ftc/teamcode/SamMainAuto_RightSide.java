package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="[Sam] *** Auto - Start RIGHT Side (MAIN) ***", group="Sam")
//@Disabled
public class SamMainAuto_RightSide extends LinearOpMode
{
    @Override
    public void runOpMode() {
        SamMainAuto_Impl impl = new SamMainAuto_Impl(this, Alliance.Side.RIGHT, true);
        impl.runOpMode();
    }
}
