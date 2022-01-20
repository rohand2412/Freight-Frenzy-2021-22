package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous (name="TestCrane", group="basic")
public class TestCrane extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.crane);

        rob.bucketRight.setPosition(0.5);
        rob.bucketLeft.setPosition(0.5);

        sleep(2000);
    }
}