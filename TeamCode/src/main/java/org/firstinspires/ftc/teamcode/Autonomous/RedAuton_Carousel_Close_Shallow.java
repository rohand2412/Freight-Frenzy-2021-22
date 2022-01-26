package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="RedAuton_Carousel_Close_Shallow", group="basic")
public class RedAuton_Carousel_Close_Shallow extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, false, Goal.setupType.autonomous);

        red(false, false);
    }
}