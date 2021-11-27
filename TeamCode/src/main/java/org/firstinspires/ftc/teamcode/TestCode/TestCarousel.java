package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="TestCarousel", group="basic")
public class TestCarousel extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.carousel);

        rob.runCarouselLeftTimeSpeed(1, 5000);
        rob.runCarouselRightTimeSpeed(1, 5000);
    }
}