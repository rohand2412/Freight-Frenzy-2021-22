package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="TestIntake", group="basic")
public class TestIntake extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.intake);

        //Suck in field objects
        rob.runIntakeTimeSpeed(-1, 4000);

        //Push out field objects
        rob.runIntakeTimeSpeed(1, 4000);
    }
}