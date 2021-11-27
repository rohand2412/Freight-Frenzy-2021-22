package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="TestImuTurn", group="basic")
public class TestImuTurn extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.autonomous);

        rob.turn(0.5, 90, Goal.axis.center);
        sleep(1000);
        rob.turn(0.5, -180, Goal.axis.center);
    }
}