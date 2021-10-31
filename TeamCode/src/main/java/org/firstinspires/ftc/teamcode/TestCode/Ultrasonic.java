package org.firstinspires.ftc.teamcode.TestCode;
//hi
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="Ultrasonic", group = "basic")
public class Ultrasonic extends AutonomousControl {

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.autonomous);
        telemetry.addLine("Start!");
        telemetry.update();
        double angle = 0;

        while (opModeIsActive()) {
            double distanceBack = rob.Right.getDistance(DistanceUnit.CM);
            double distanceFront = rob.Right.getDistance(DistanceUnit.CM);
            telemetry.addData("right", "%.2f cm", rob.Right.getDistance(DistanceUnit.CM));
            telemetry.addData("front", "%.2f cm", rob.Front.getDistance(DistanceUnit.CM));
            telemetry.addData("back", "%.2f cm", rob.Back.getDistance(DistanceUnit.CM));
            telemetry.addData("left", "%.2f cm", rob.Left.getDistance(DistanceUnit.CM));

            telemetry.addData("difference", "%.2f cm", Math.abs(distanceBack - distanceFront));
            telemetry.addData("angle", "%.2f",(Math.atan((distanceFront-distanceBack)/6.6142)*180)/(3.1415));
            telemetry.update();

//            while (rob.Back.getDistance((DistanceUnit.CM))< 156){
//                rob.driveTrainMovement(.5, Goal.movements.forward);
//            }

          /*  if (distanceBack > 1000 || distanceFront > 1000) {
                continue;
            }
            angle = (Math.atan((distanceFront - distanceBack) / 6.6142) * 180) / (3.1415);
            rob.driveTrainEncoderMovement(.5, 12.75 / 90 * angle, 10, 10, Goal.movements.ccw);

            break;

           */

/*
            if (distance1 > 1000 || distance2 > 1000) {
                continue;
            }
            else {
                if (Math.abs(distance1 - distance2) < 0.5) {
                    rob.stopDrivetrain();
                    break;
                } else {
                   if(Math.abs(distance1 - distance2) >= 5) {
                        if (rob.rightBack.getDistance(DistanceUnit.CM) > rob.rightFront.getDistance(DistanceUnit.CM)) {
                            rob.driveTrainMovement(.3, Goal.movements.ccw);
                        } else {
                            rob.driveTrainMovement(.3, Goal.movements.cw);
                        }
                    }

                    else{
                        if (rob.rightBack.getDistance(DistanceUnit.CM) > rob.rightFront.getDistance(DistanceUnit.CM)) {
                            rob.driveTrainMovement(.1, Goal.movements.ccw);
                        } else {
                            rob.driveTrainMovement(.1, Goal.movements.cw);
                        }
                    }



                }

            }

        */
        }

    }
}

 /*   double dist = 0;
            do{
                    rob.driveTrainMovement(0.3, Goal.movements.backward);

                    dist= rob.rightFront.getDistance(DistanceUnit.CM);
                    telemetry.addData("cm front", "%.2f cm", dist);
                    telemetry.update();

                    }
                    while(dist > 20 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());

  */