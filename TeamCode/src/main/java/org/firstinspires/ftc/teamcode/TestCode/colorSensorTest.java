package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="ColorSensor", group = "basic")

public class colorSensorTest extends AutonomousControl {
    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.autonomous);
        telemetry.addLine("Start!");
        telemetry.update();

        while (opModeIsActive()) {
            /*
            telemetry.addData("Red  ", rob.color1.red());
            telemetry.addData("Green", rob.color1.green());
            telemetry.addData("Blue ", rob.color1.blue());
            telemetry.update();

            while ((rob.color1.red() < 40  && rob.color1.green() < 40 && rob.color1.blue() < 40) || (rob.color1.red() > 50 && rob.color1.green() < 40 && rob.color1.blue() < 40)){
                rob.driveTrainMovement(1, Goal.movements.forward);

             */

            }


        }
    }

