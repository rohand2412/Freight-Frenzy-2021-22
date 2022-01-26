package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.Constants;

@Autonomous (name="TestCranePivot", group="basic")
public class TestCranePivot extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.crane, Goal.setupType.bucket);

        rob.bucketSetPosition(0.51);
        for (int i = 0; i < 20; i++) {
            rob.pivotCrane(0.2, 180);
            rob.pivotCrane(0.2, -180);
        }
    }
}