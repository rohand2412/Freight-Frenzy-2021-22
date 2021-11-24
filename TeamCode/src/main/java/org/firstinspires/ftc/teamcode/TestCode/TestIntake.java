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

        rob.intakeClaw.setPosition(0);

//        //Suck in field objects
        rob.runIntakeTimeSpeed(-1, 4000);
//
//        //Push out field objects
//        rob.runIntakeTimeSpeed(1, 4000);

        rob.intakeClaw.setPosition(1);
//        rob.cappingPivot.setPosition(0.65);
        sleep(2000);
    }
}