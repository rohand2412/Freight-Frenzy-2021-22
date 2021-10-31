package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="Test Intake", group = "basic")
public class TestIntake extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.collectionsystem);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            rob.collection.setPower(1);
            //sleep(5000);
            sleep(30000);
        }


    }
}
