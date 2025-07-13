package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="[Sam] (1) Start RIGHT Side (CLIP+PUSH)", group="Sam")
@Disabled
public class SamMainAuto_RightSide extends LinearOpMode
{
    @Override
    public void runOpMode() {
        SamMainAuto_Impl impl = new SamMainAuto_Impl(this, Alliance.Side.RIGHT, true, true);
        impl.runOpMode();
    }
}
