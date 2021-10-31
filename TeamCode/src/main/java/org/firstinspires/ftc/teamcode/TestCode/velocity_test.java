package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

import java.util.ArrayList;
import java.util.Locale;

@Autonomous(name="velocity", group = "basic")

public class velocity_test extends AutonomousControl {


    @Override

    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.autonomous);
        telemetry.addLine("Start!");

        telemetry.update();
        if (opModeIsActive()) {

            rob.lifter.setPosition(.85);
            sleep(500);

            rob.fly.setPower(-.7375);
            sleep(4000);

            //rob.fly.setPower(-.65);
            for(int i = 0; i<100; i++) {

                rob.PIDFly(3, -.7375, 1158, .1, .03, 0, .15, .001, 0);

 /*                double val = rob.velocityFly(-.73, 4);
                telemetry.addData("velocity", "%.2f", val);
                telemetry.update();
*/

                rob.whack.setPosition(0.45);
                sleep(500);


                rob.whack.setPosition(0);
                sleep(500);


            }

 /*           while (true){

                rob.fly.setPower(-.65);
                sleep(3000);
                rob.lifter.setPosition(.86);
                sleep(500);
                for (int i = 0; i <= 100; i++) {

                    telemetry.addData("Velocity", rob.velocityFly());
                    telemetry.update();

                    rob.whack.setPosition(0.45);
                    sleep(500);


                    rob.whack.setPosition(0);
                    sleep(750);

                    rob.fly.setPower(-.65);
                    sleep(3000);

                }
*/
               telemetry.update();

            }






            //  rob.driveTrainIMUSwingTurnMovement(0.4, Goal.movements.backward, 3000, 90, 0.02, Goal.turnside.cw);


        }
    }



