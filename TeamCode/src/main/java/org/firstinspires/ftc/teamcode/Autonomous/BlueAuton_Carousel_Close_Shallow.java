package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="BlueAuton_Carousel_Close_Shallow", group="basic")
public class BlueAuton_Carousel_Close_Shallow extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.autonomous);

        if (opModeIsActive()) blue(true, true, true);
    }
}