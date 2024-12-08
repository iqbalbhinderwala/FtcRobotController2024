package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="[Sam] (4) Start LEFT Side (CLIP+PUSH)", group="Sam")
//@Disabled
public class SamMainAuto_LeftSide_ClipPush extends LinearOpMode
{
    @Override
    public void runOpMode() {
        SamMainAuto_Impl impl = new SamMainAuto_Impl(this, Alliance.Side.LEFT, true, true);
        impl.runOpMode();
    }
}
