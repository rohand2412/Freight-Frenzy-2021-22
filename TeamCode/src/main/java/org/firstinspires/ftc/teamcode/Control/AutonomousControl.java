package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class AutonomousControl extends Central {

    public boolean outlier(){
//        if(rob.rightBack.getDistance(DistanceUnit.CM) >= 1000 ||
//                rob.rightFront.getDistance(DistanceUnit.CM) >= 1000 ||
//                rob.Back.getDistance(DistanceUnit.CM) >= 1000){
//            return true;
//        }
//        else{
//            return false;
//        }
        if(rob.Right.getDistance(DistanceUnit.CM) >= 1000 ||
                rob.Front.getDistance(DistanceUnit.CM) >= 1000 ||
                rob.Left.getDistance(DistanceUnit.CM) >= 1000 ||
                rob.Back.getDistance(DistanceUnit.CM) >= 1000){
            return true;
        }
        else{
            return false;
        }
    }

    public void zero() throws InterruptedException {
        //move to red square
        rob.driveTrainEncoderMovement(1, 66, 20, 0, Goal.movements.forward);
        // previous 63

        //drop wobble goal
        rob.pinch.setPosition(0.8);
        sleep(500);
        rob.claw.setPower(0.3);
        sleep(250);
        rob.claw.setPower(0);
        sleep(250);

        //move back to pick up second wobble goal

        rob.driveTrainEncoderMovement(1, 46, 20, 0, Goal.movements.backward);

        rob.lifter.setPosition(.87);
        sleep(200);

        rob.driveTrainEncoderMovement(1, 6, 20, 0, Goal.movements.left);

        // turn to face second wobble goal
        rob.driveTrainEncoderMovement(1, 23, 20, 0, Goal.movements.cw);

        rob.pinch.setPosition(0.8);
        sleep(500);

        // move to second wobble goal
        rob.driveTrainEncoderMovement(.75, 16, 20, 0, Goal.movements.forward);

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
        rob.driveTrainEncoderMovement(1, 23, 20, 0, Goal.movements.ccw);
        rob.driveTrainEncoderMovement(1, 23, 20, 0, Goal.movements.right);
        rob.driveTrainEncoderMovement(1, 42, 20, 0, Goal.movements.forward);

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


        // correct angle
//            angles = rob.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            telemetry.addData("angle", (int)angles.firstAngle);
//            telemetry.update();
//            sleep(3000);
//            rob.driveTrainIMUSwingTurnMovement(0.4, Goal.movements.backward, 3000, (int)angles.firstAngle, 0.02, Goal.turnside.cw);

//           rob.driveTrainEncoderMovement(1, 5, 20, 0, Goal.movements.backward);
        // move to shooting position
//            rob.driveTrainEncoderMovement(1, (rob.rightFront.getDistance(DistanceUnit.CM) - 46) / 2.54, 20, 0, Goal.movements.right);
//            rob.driveTrainEncoderMovement(1, (46-rob.rightFront.getDistance(DistanceUnit.CM)) / 2.54, 20, 0, Goal.movements.left);
//            close_movement(0, 46);
//            rob.stopDrivetrain();
//            rob.driveTrainEncoderMovement(1,(rob.Back.getDistance(DistanceUnit.CM) - 150)/2.54,20,0,Goal.movements.backward);
//            close_movement(1, 150);
//            rob.stopDrivetrain();

        // move backwards a bit so you dont hit the wobble goal
        rob.driveTrainEncoderMovement(1, 7, 20, 0, Goal.movements.backward);

        // move towards wall
        rob.driveTrainEncoderMovement(1, 9, 20, 0, Goal.movements.right);

        // move to the left, to align shots
        rob.driveTrainEncoderMovement(1, 22, 20, 0, Goal.movements.left);

        // move to right behind white line
        rob.driveTrainEncoderMovement(1, 9, 20, 0, Goal.movements.forward);

        // shoot your shots

        sleep(500);
        for (int i = 0; i <= 2; i++) {
            rob.fly.setPower(-0.73);
            sleep(200);
            //200
            rob.whack.setPosition(0.4);
            sleep(1000);
            //1000 each
            rob.whack.setPosition(0);
            sleep(1000);
        }

        // move to Launch Line
        rob.driveTrainEncoderMovement(1, 8, 100, 100, Goal.movements.forward);
    }

    public void one() throws InterruptedException {
        // 1 ring
        //move to red square
        rob.driveTrainEncoderMovement(1,90,20,0,Goal.movements.forward);
        rob.driveTrainEncoderMovement(1,20,20,0,Goal.movements.left);

        //drop wobble goal
        rob.pinch.setPosition(0.8);
        sleep(500);
        rob.claw.setPower(0.3);
        sleep(250);
        rob.claw.setPower(0);
        sleep(250);

        //move back to pick up second wobble goal
        rob.driveTrainEncoderMovement(1,5,20,0,Goal.movements.backward);
        rob.driveTrainEncoderMovement(1,12,20,0,Goal.movements.right);
        rob.driveTrainEncoderMovement(1,65,20,0,Goal.movements.backward);
        rob.lifter.setPosition(.87);
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

        // move backwards a bit so you dont hit the wobble goal
        rob.driveTrainEncoderMovement(1, 31, 20, 0, Goal.movements.backward);

        // move towards wall
        rob.driveTrainEncoderMovement(1, 29, 20, 0, Goal.movements.right);

        // move to the left, to align shots
        rob.driveTrainEncoderMovement(1, 22, 20, 0, Goal.movements.left);

        // move to right behind white line
        rob.driveTrainEncoderMovement(1, 9, 20, 0, Goal.movements.forward);

        // shoot your shots
        sleep(500);
        for (int i = 0; i <= 2; i++) {
            rob.fly.setPower(-0.73);
            sleep(200);
            //200
            rob.whack.setPosition(0.4);
            sleep(1000);
            //1000 each
            rob.whack.setPosition(0);
            sleep(1000);
        }

        // move to Launch Line
        rob.driveTrainEncoderMovement(1, 8, 100, 100, Goal.movements.forward);
    }

    public void four() throws InterruptedException {
        //move to red square
        rob.driveTrainEncoderMovement(1,102,20,0,Goal.movements.forward);

        //drop wobble goal
        rob.pinch.setPosition(0.8);
        sleep(500);
        rob.claw.setPower(0.3);
        sleep(250);
        rob.claw.setPower(0);
        sleep(250);

        //move back to pick up second wobble goal

        rob.driveTrainEncoderMovement(1,82,20,0,Goal.movements.backward);

        rob.lifter.setPosition(.87);
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

        // move backwards a bit so you dont hit the wobble goal
        rob.driveTrainEncoderMovement(1, 43, 20, 0, Goal.movements.backward);

        // move towards wall
        rob.driveTrainEncoderMovement(1, 9, 20, 0, Goal.movements.right);

        // move to the left, to align shots
        rob.driveTrainEncoderMovement(1, 22, 20, 0, Goal.movements.left);

        // move to right behind white line
        rob.driveTrainEncoderMovement(1, 9, 20, 0, Goal.movements.forward);

        // shoot your shots

        sleep(500);
        for (int i = 0; i <= 2; i++) {
            rob.fly.setPower(-0.73);
            sleep(200);
            //200
            rob.whack.setPosition(0.4);
            sleep(1000);
            //1000 each
            rob.whack.setPosition(0);
            sleep(1000);
        }

        // move to Launch Line
        rob.driveTrainEncoderMovement(1, 8, 100, 100, Goal.movements.forward);
    }
    public void dropgoal() {
        //drop wobble goal
        rob.pinch.setPosition(0.8);
        sleep(200);
        rob.claw.setPower(0.4);
        sleep(200);
        rob.claw.setPower(0);
        sleep(100);
    }

    public void pickupgoal() {
        rob.claw.setPower(-0.4);
        sleep(300);
        rob.claw.setPower(0);
        sleep(100);
        rob.pinch.setPosition(0);
        sleep(400);
    }

    

}






