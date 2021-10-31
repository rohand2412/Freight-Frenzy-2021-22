package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Auto w/out openCV)", group = "basic")

public class AutoNoCV extends AutonomousControl {

    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Goal.setupType.autonomous);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()) {
           sleep(3000);
            angles = rob.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("angle", angles.firstAngle);
            telemetry.update();
            sleep(4000);
            rob.driveTrainIMUSwingTurnMovement(0.4, Goal.movements.backward, 3000, (int)angles.firstAngle, 0.02, Goal.turnside.cw);
            /*
            telemetry.update();
            // pick up wobble goal
             rob.claw.setPower(-0.4);
          sleep(750);
        rob.claw.setPower(0);
            sleep(250);

          */
          /*  telemetry.addData("distance to wall", rob.rightFront.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(3000);
            rob.driveTrainEncoderMovement(.2, (rob.rightFront.getDistance(DistanceUnit.CM) - 46) / 2.54, 20, 0, Goal.movements.right);
            sleep(200);

            double dist = rob.rightBack.getDistance(DistanceUnit.CM);
           while (dist > 1000 || dist > 46 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive()) {
                rob.driveTrainMovement(0.1, Goal.movements.right);
                dist = rob.rightBack.getDistance(DistanceUnit.CM);
                telemetry.addData("cm Back", "%.2f cm", dist);
                telemetry.update();
            }*/

            rob.stopDrivetrain();
          //  rob.driveTrainEncoderMovement(.2,(rob.Back.getDistance(DistanceUnit.CM) - 153)/2.54,20,0,Goal.movements.backward);




/* 0 rings
            //move to red square
            rob.driveTrainEncoderMovement(1,66,20,0,Goal.movements.forward);
            // previous 63

            //drop wobble goal
            rob.pinch.setPosition(0.8);
            sleep(500);
            rob.claw.setPower(0.3);
            sleep(500);
            rob.claw.setPower(0);
            sleep(250);

            //move back to pick up second wobble goal

            rob.driveTrainEncoderMovement(1,46,20,0,Goal.movements.backward);

            rob.lifter.setPosition(.84);
            sleep(200);

            rob.driveTrainEncoderMovement(1,6,20,0,Goal.movements.left);

            // turn to face second wobble goal
            rob.driveTrainEncoderMovement(1,23,20,0,Goal.movements.cw);

            rob.pinch.setPosition(0.8);
            sleep(500);

            // move to second wobble goal
            rob.driveTrainEncoderMovement(.75,16,20,0,Goal.movements.forward);

            // pick up second wobble goal
            sleep(250);
            rob.claw.setPower(-0.3);
            sleep(300);
            rob.claw.setPower(0);
            sleep(500);
            rob.pinch.setPosition(0);
            sleep(400);
            rob.claw.setPower(-0.4);
            sleep(350);
            rob.claw.setPower(0);
            sleep(250);

            // move backwards to go back to red square
            rob.driveTrainEncoderMovement(1,23,20,0,Goal.movements.ccw);
            rob.driveTrainEncoderMovement(1,23,20,0,Goal.movements.right);
            rob.driveTrainEncoderMovement(1,42,20,0,Goal.movements.forward);

            //drop wobble goal
            rob.pinch.setPosition(0.8);
            sleep(250);
            rob.claw.setPower(0.3);
            sleep(500);
            rob.claw.setPower(0);
            sleep(250);

            // start flywheel
            rob.fly.setPower(-0.73);
            sleep(2000);

            // move backwards a bit so you dont hit the wobble goal
            rob.driveTrainEncoderMovement(1,7,20,0,Goal.movements.backward);

            // move towards wall
            // rob.driveTrainEncoderMovement(1,9,20, 0, Goal.movements.right);

            // move to the left, to align shots
            rob.driveTrainEncoderMovement(1,22 ,20,0,Goal.movements.left);

            // move to right behind white line
            rob.driveTrainEncoderMovement(1,9,20,0,Goal.movements.forward);

            // shoot your shots

            sleep(500);
            for(int i = 0; i<=2; i++) {
                rob.fly.setPower(-0.73);
                sleep(200);
                //200
                rob.whack.setPosition(0.62);
                sleep(1000);
                //1000 each
                rob.whack.setPosition(0);
                sleep(1000);
            }

            // move to Launch Line
            rob.driveTrainEncoderMovement(1,8, 100, 100,Goal.movements.forward);
            */
/*
            // 1 ring
            //move to red square
            rob.driveTrainEncoderMovement(1,90,20,0,Goal.movements.forward);
            rob.driveTrainEncoderMovement(1,20,20,0,Goal.movements.left);

            //drop wobble goal
            rob.pinch.setPosition(0.8);
            sleep(500);
            rob.claw.setPower(0.3);
            sleep(500);
            rob.claw.setPower(0);
            sleep(250);

            //move back to pick up second wobble goal
            rob.driveTrainEncoderMovement(1,5,20,0,Goal.movements.backward);
            rob.driveTrainEncoderMovement(1,12,20,0,Goal.movements.right);
            rob.driveTrainEncoderMovement(1,65,20,0,Goal.movements.backward);
            rob.lifter.setPosition(.84);
            sleep(200);

            // turn to face second wobble goal
            rob.driveTrainEncoderMovement(1,23,20,0,Goal.movements.cw);

            rob.pinch.setPosition(0.8);
            sleep(500);

            // move to second wobble goal
            rob.driveTrainEncoderMovement(.75,16,20,0,Goal.movements.forward);

            // pick up second wobble goal
            sleep(250);
            rob.claw.setPower(-0.3);
            sleep(300);
            rob.claw.setPower(0);
            sleep(500);
            rob.pinch.setPosition(0);
            sleep(400);
            rob.claw.setPower(-0.4);
            sleep(350);
            rob.claw.setPower(0);
            sleep(250);

            //return to red square
            rob.driveTrainEncoderMovement(1,23,20,0,Goal.movements.ccw);
            rob.driveTrainEncoderMovement(.75,70,20,0,Goal.movements.forward);

            //drop wobble goal
            rob.pinch.setPosition(0.8);
            sleep(500);
            rob.claw.setPower(0.3);
            sleep(500);
            rob.claw.setPower(0);
            sleep(250);
            telemetry.addData("back",rob.Back.getDistance(DistanceUnit.CM) );

//            while (rob.Back.getDistance(DistanceUnit.CM) > 100){
//                rob.driveTrainMovement(1, Goal.movements.backward);
//            }
            double dist = 0;
            do{
                rob.driveTrainMovement(1, Goal.movements.backward);

                dist = rob.Back.getDistance(DistanceUnit.CM);
                telemetry.addData("cm Back", "%.2f cm", dist);
                telemetry.update();

            }
            while(dist >1000 || dist > 153 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());

            rob.stopDrivetrain();
            angles = rob.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            rob.driveTrainIMUSwingTurnMovement(0.1, Goal.movements.backward, 300, (int)angles.firstAngle, 0.02, Goal.turnside.cw);

//            while(rob.rightBack.getDistance(DistanceUnit.CM) > 20){
//            }
            do{
                rob.driveTrainMovement(1, Goal.movements.right);

                dist = rob.Back.getDistance(DistanceUnit.CM);
                telemetry.addData("cm Back", "%.2f cm", dist);
                telemetry.update();

            }
            while(dist >1000 || dist > 46 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
            rob.stopDrivetrain();
 */

            /* 4 rings
            //move to red square
            rob.driveTrainEncoderMovement(1,102,20,0,Goal.movements.forward);

            //drop wobble goal
            rob.pinch.setPosition(0.8);
            sleep(500);
            rob.claw.setPower(0.3);
            sleep(500);
            rob.claw.setPower(0);
            sleep(250);

            //move back to pick up second wobble goal

            rob.driveTrainEncoderMovement(1,82,20,0,Goal.movements.backward);

            rob.lifter.setPosition(.84);
            sleep(200);

            rob.driveTrainEncoderMovement(1,6,20,0,Goal.movements.left);

            // turn to face second wobble goal
            rob.driveTrainEncoderMovement(1,23,20,0,Goal.movements.cw);

            rob.pinch.setPosition(0.8);
            sleep(500);

            // move to second wobble goal
            rob.driveTrainEncoderMovement(.75,16,20,0,Goal.movements.forward);

            // pick up second wobble goal
            sleep(250);
            rob.claw.setPower(-0.3);
            sleep(300);
            rob.claw.setPower(0);
            sleep(500);
            rob.pinch.setPosition(0);
            sleep(400);
            rob.claw.setPower(-0.4);
            sleep(350);
            rob.claw.setPower(0);
            sleep(250);

            // move backwards to go back to red square
            rob.driveTrainEncoderMovement(1,23,20,0,Goal.movements.ccw);
            rob.driveTrainEncoderMovement(1,23,20,0,Goal.movements.right);
            rob.driveTrainEncoderMovement(1,82,20,0,Goal.movements.forward);

            //drop wobble goal
            rob.pinch.setPosition(0.8);
            sleep(250);
            rob.claw.setPower(0.3);
            sleep(500);
            rob.claw.setPower(0);
            sleep(250);

             */


        }
    }
}

    /*
    public void close_movement(int dir, double d) throws InterruptedException {
        double speed = 0.2;
        if (dir == 0) {
            double dist = rob.rightFront.getDistance(DistanceUnit.CM);
            while (dist > 1000 || dist > d || Double.compare(dist, Double.NaN) == 0 && opModeIsActive()) {
                rob.driveTrainMovement(speed, Goal.movements.right);
                dist = rob.rightFront.getDistance(DistanceUnit.CM);
                telemetry.addData("cm Back", "%.2f cm", dist);
                telemetry.update();
            }
            while (dist > 1000 || dist < d || Double.compare(dist, Double.NaN) == 0 && opModeIsActive()) {
                rob.driveTrainMovement(speed, Goal.movements.left);
                dist = rob.rightFront.getDistance(DistanceUnit.CM);
                telemetry.addData("cm Back", "%.2f cm", dist);
                telemetry.update();
            }
        } else {
            double dist = rob.Back.getDistance(DistanceUnit.CM);
            while (dist > 1000 || dist > d || Double.compare(dist, Double.NaN) == 0 && opModeIsActive()) {
                rob.driveTrainMovement(speed, Goal.movements.backward);
                dist = rob.Back.getDistance(DistanceUnit.CM);
                telemetry.addData("cm Back", "%.2f cm", dist);
                telemetry.update();
            }
            while (dist > 1000 || dist < d || Double.compare(dist, Double.NaN) == 0 && opModeIsActive()) {
                rob.driveTrainMovement(speed, Goal.movements.forward);
                dist = rob.Back.getDistance(DistanceUnit.CM);
                telemetry.addData("cm Back", "%.2f cm", dist);
                telemetry.update();
            }
        }
    }

     */
