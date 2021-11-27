package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import static org.firstinspires.ftc.teamcode.Control.Constants.LABEL_FIRST_ELEMENT;
import static org.firstinspires.ftc.teamcode.Control.Constants.LABEL_SECOND_ELEMENT;

public abstract class AutonomousControl extends Central {
    static double speed = 0.5;
    static long timeoutS = 10000;
    static long waitAfter = 0;
    locations location;

    public void blue(boolean doingCarousel) throws InterruptedException {
        rob.driveTrainEncoderMovement(speed, 6, timeoutS, waitAfter, Goal.movements.backward);
        determineLocation();

        if (doingCarousel) {
            rob.driveTrainEncoderMovement(speed, 20, timeoutS, waitAfter, Goal.movements.cw);
            rob.driveTrainEncoderMovement(speed, 25, timeoutS, waitAfter, Goal.movements.forward);
            rob.driveTrainEncoderMovement(speed, 7, timeoutS, waitAfter, Goal.movements.right);
            rob.runCarouselLeftTimeSpeed(0.25, 6000);
            rob.runCarouselRightTimeSpeed(0.25, 6000);
            rob.driveTrainEncoderMovement(speed, 60, timeoutS, waitAfter, Goal.movements.backward);
            rob.driveTrainEncoderMovement(speed, 20, timeoutS, waitAfter, Goal.movements.ccw);
            rob.cappingPivot.setPosition(0.65);
            rob.moveLinearSlideInches(1, location.getLinearSlideInches(), rob.cappingLinearSlide);
            rob.driveTrainEncoderMovement(speed, 20, timeoutS, waitAfter, Goal.movements.backward);
            rob.cappingClaw.setPosition(0.2);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 37, timeoutS, waitAfter, Goal.movements.left);
            rob.cappingPivot.setPosition(0.65);
            rob.moveLinearSlideInches(1, location.getLinearSlideInches(), rob.cappingLinearSlide);
            rob.driveTrainEncoderMovement(speed, 15, timeoutS, waitAfter, Goal.movements.backward);
            rob.cappingClaw.setPosition(0.2);
        }

        sleep(500);
        rob.driveTrainEncoderMovement(speed, 12, timeoutS, waitAfter, Goal.movements.forward);
        rob.moveLinearSlideInches(1, -location.getLinearSlideInches(), rob.cappingLinearSlide);
        rob.cappingClaw.setPosition(0.045);
        rob.cappingPivot.setPosition(0.2);
        rob.driveTrainEncoderMovement(speed, 20, timeoutS, waitAfter, Goal.movements.cw);
        rob.driveTrainEncoderMovement(speed, 26, timeoutS, waitAfter, Goal.movements.right);
        rob.driveTrainEncoderMovement(speed, 65, timeoutS, waitAfter, Goal.movements.backward);
        rob.driveTrainEncoderMovement(speed, 40, timeoutS, waitAfter, Goal.movements.left);
        rob.driveTrainEncoderMovement(speed, 25, timeoutS, waitAfter, Goal.movements.backward);
    }

    public void red(boolean doingCarousel) throws InterruptedException {
        rob.driveTrainEncoderMovement(speed, 7, timeoutS, waitAfter, Goal.movements.backward);
        determineLocation();

        if (doingCarousel) {
            rob.driveTrainEncoderMovement(speed, 40, timeoutS, waitAfter, Goal.movements.right);
            rob.driveTrainEncoderMovement(speed, 0.5, timeoutS, waitAfter, Goal.movements.forward);
            rob.runCarouselLeftTimeSpeed(-0.25, 6000);
            rob.runCarouselRightTimeSpeed(-0.25, 6000);
            rob.driveTrainEncoderMovement(speed, 75, timeoutS, waitAfter, Goal.movements.left);
            rob.cappingPivot.setPosition(0.65);
            rob.moveLinearSlideInches(1, location.getLinearSlideInches(), rob.cappingLinearSlide);
            rob.driveTrainEncoderMovement(speed, 16, timeoutS, waitAfter, Goal.movements.backward);
            rob.cappingClaw.setPosition(0.2);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 37, timeoutS, waitAfter, Goal.movements.right);
            rob.cappingPivot.setPosition(0.65);
            rob.moveLinearSlideInches(1, location.getLinearSlideInches(), rob.cappingLinearSlide);
            rob.driveTrainEncoderMovement(speed, 15, timeoutS, waitAfter, Goal.movements.backward);
            rob.cappingClaw.setPosition(0.2);
        }

        sleep(500);
        rob.driveTrainEncoderMovement(speed, 12, timeoutS, waitAfter, Goal.movements.forward);
        rob.moveLinearSlideInches(1, -location.getLinearSlideInches(), rob.cappingLinearSlide);
        rob.cappingClaw.setPosition(0.045);
        rob.cappingPivot.setPosition(0.2);
        rob.driveTrainEncoderMovement(speed, 20, timeoutS, waitAfter, Goal.movements.cw);
        rob.driveTrainEncoderMovement(speed, 19, timeoutS, waitAfter, Goal.movements.right);
        rob.driveTrainEncoderMovement(speed, 70, timeoutS, waitAfter, Goal.movements.forward);
        rob.driveTrainEncoderMovement(speed, 40, timeoutS, waitAfter, Goal.movements.left);
        rob.driveTrainEncoderMovement(speed, 25, timeoutS, waitAfter, Goal.movements.forward);
    }

    public void determineLocation() throws InterruptedException {
        if (rob.tfod != null) {
            sleep(1000);
            List<Recognition> finalRecognitions = null;
            for (int i = 0; i < 6; i++) {
                List<Recognition> recognitions = rob.tfod.getUpdatedRecognitions();
                if (recognitions.size() > 0) { finalRecognitions = recognitions; }
                rob.driveTrainEncoderMovement(speed, 1, timeoutS, waitAfter, Goal.movements.backward);
            }

            if (finalRecognitions == null) location = locations.left;
            else if (finalRecognitions.size() == 0) location = locations.left;
            else if ((finalRecognitions.get(0).getLeft() + finalRecognitions.get(0).getRight())/2 < 320) location = locations.middle;
            else location = locations.right;

            telemetry.addLine(location.toString());
            telemetry.update();
        }
    }

    public enum locations {
        left("left", 5.5),
        middle("middle", -1),
        right("right", -5.75);

        private final String name;
        private final double linearSlideInches;

        locations(String name, double linearSlideInches) { this.name = name; this.linearSlideInches = linearSlideInches; }

        public String toString() { return this.name; }

        public double getLinearSlideInches() { return this.linearSlideInches; }
    }
}






