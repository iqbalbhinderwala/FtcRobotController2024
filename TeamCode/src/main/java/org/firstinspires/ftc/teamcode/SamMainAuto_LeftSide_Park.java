package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="[Sam] (3) Start LEFT Side (PARK)", group="Sam")
//@Disabled
public class SamMainAuto_LeftSide_Park extends LinearOpMode
{
    @Override
    public void runOpMode() {
        SamMainAuto_Impl impl = new SamMainAuto_Impl(this, Alliance.Side.LEFT, false);
        impl.runOpMode();
    }
}
