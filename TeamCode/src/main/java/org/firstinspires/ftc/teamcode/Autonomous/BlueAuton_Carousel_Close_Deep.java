package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="BlueAuton_Carousel_Close_Deep", group="basic")
public class BlueAuton_Carousel_Close_Deep extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, false, Goal.setupType.autonomous);

        determineLocation(true);
        waitAndStart();

        if (opModeIsActive()) blue(true, true, false);
    }
}