package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="TestLinearSlide", group="basic")
public class TestLinearSlide extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.linear_slide);

        rob.rotate.setPosition(1);
        rob.claw.setPosition(0.2);

        rob.moveLinearSlideInches(1, 1);
        rob.moveLinearSlideInches(1, -1);
    }
}