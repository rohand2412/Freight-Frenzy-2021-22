package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="TestCapping", group="basic")
public class TestCapping extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.capping);

//        rob.rotate.setPosition(1);
//        rob.claw.setPosition(0.2);

//        rob.moveLinearSlideInches(1, 1);
//        rob.moveLinearSlideInches(1, -1);
        rob.cappingClaw.setPosition(0.1);
        sleep(2000);
    }
}