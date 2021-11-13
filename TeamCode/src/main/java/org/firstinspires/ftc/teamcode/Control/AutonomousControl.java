package org.firstinspires.ftc.teamcode.Control;

public abstract class AutonomousControl extends Central {
    static double speed = 0.5;
    static long timeoutS = 10000;
    static long waitAfter = 0;

    public void blue(boolean doingCarousel) throws InterruptedException {
        if (doingCarousel) {
            rob.driveTrainEncoderMovement(speed, 6, timeoutS, waitAfter, Goal.movements.forward);
            rob.driveTrainEncoderMovement(speed, 12, timeoutS, waitAfter, Goal.movements.right);
            rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, Goal.movements.ccw);
            rob.runCarouselTimeSpeed(1, 1500);
            rob.driveTrainEncoderMovement(speed, 12, timeoutS, waitAfter, Goal.movements.backward);
            rob.driveTrainEncoderMovement(speed, 6, timeoutS, waitAfter, Goal.movements.ccw);
            rob.driveTrainEncoderMovement(speed, 48, timeoutS, waitAfter, Goal.movements.backward);
            rob.driveTrainEncoderMovement(speed, 28, timeoutS, waitAfter, Goal.movements.ccw);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 4, timeoutS, waitAfter, Goal.movements.backward);
        }

        rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, Goal.movements.backward);
        rob.rotate.setPosition(0.5);
        sleep(500);
        rob.claw.setPosition(0.2);
        sleep(500);
        rob.rotate.setPosition(0);
        rob.claw.setPosition(0);
        rob.driveTrainEncoderMovement(speed, 20, timeoutS, waitAfter, Goal.movements.forward);
        rob.driveTrainEncoderMovement(speed, 28, timeoutS, waitAfter, Goal.movements.ccw);
        rob.driveTrainEncoderMovement(speed/2, 84, timeoutS, waitAfter, Goal.movements.forward);
        rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, Goal.movements.right);
        rob.moveIntakePivotDegrees(-1, -90);
    }

    public void red(boolean doingCarousel) throws InterruptedException {
        if (doingCarousel) {
            rob.driveTrainEncoderMovement(speed, 6, timeoutS, waitAfter, Goal.movements.right);
            rob.driveTrainEncoderMovement(speed, 12, timeoutS, waitAfter, Goal.movements.forward);
            rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, Goal.movements.cw);
            rob.runCarouselTimeSpeed(-1, 1500);
            rob.driveTrainEncoderMovement(speed, 12, timeoutS, waitAfter, Goal.movements.left);
            rob.driveTrainEncoderMovement(speed, 6, timeoutS, waitAfter, Goal.movements.cw);
            rob.driveTrainEncoderMovement(speed, 48, timeoutS, waitAfter, Goal.movements.left);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 4, timeoutS, waitAfter, Goal.movements.backward);
        }

        rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, Goal.movements.backward);
        rob.rotate.setPosition(0.5);
        sleep(500);
        rob.claw.setPosition(0.2);
        sleep(500);
        rob.rotate.setPosition(0);
        rob.claw.setPosition(0);
        rob.driveTrainEncoderMovement(speed, 20, timeoutS, waitAfter, Goal.movements.forward);
        rob.driveTrainEncoderMovement(speed, 28, timeoutS, waitAfter, Goal.movements.cw);
        rob.driveTrainEncoderMovement(speed/2, 84, timeoutS, waitAfter, Goal.movements.forward);
        rob.driveTrainEncoderMovement(speed, 28, timeoutS, waitAfter, Goal.movements.left);
        rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, Goal.movements.forward);
        rob.moveIntakePivotDegrees(-1, -90);
    }
}






