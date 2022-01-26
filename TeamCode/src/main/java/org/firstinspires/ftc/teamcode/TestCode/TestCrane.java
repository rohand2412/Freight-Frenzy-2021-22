package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.Constants;

@Autonomous (name="TestCrane", group="basic")
public class TestCrane extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.crane, Goal.setupType.bucket);

        rob.liftCraneHoldBucket(0.1, 90);

        sleep(2000);
    }
}