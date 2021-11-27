package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="BlueAuton_NoCarousel_Far_Deep", group="basic")
public class BlueAuton_NoCarousel_Far_Deep extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.autonomous);

        if (opModeIsActive()) blue(false, false, false);
    }
}