package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="RedAuton_NoCarousel_Far_Shallow", group="basic")
public class RedAuton_NoCarousel_Far_Shallow extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.autonomous);

        if (opModeIsActive()) red(false, false);
    }
}