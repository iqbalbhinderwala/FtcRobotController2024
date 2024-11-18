package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="[Sam] ** ANY Alliance - LEFT Side ** ", group="Sam")
//@Disabled
public class SamMainAuto_LeftSide extends LinearOpMode
{
    @Override
    public void runOpMode() {
        SamMainAuto_Impl impl = new SamMainAuto_Impl(this, Alliance.Side.LEFT);
        impl.runOpMode();
    }
}