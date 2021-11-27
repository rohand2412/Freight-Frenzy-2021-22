package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import static org.firstinspires.ftc.teamcode.Control.Constants.LABEL_FIRST_ELEMENT;
import static org.firstinspires.ftc.teamcode.Control.Constants.LABEL_SECOND_ELEMENT;

public abstract class AutonomousControl extends Central {
    static double speed = 0.5;
    locations location;

    public void blue(boolean doingCarousel) throws InterruptedException {
        rob.driveTrainEncoderMovement(speed, 6, Goal.movements.backward);
        determineLocation();

        if (doingCarousel) {
            rob.driveTrainEncoderMovement(speed, 20, Goal.movements.cw);
            rob.driveTrainEncoderMovement(speed, 25, Goal.movements.forward);
            rob.driveTrainEncoderMovement(speed, 7, Goal.movements.right);
            rob.runCarouselLeftTimeSpeed(0.25, 6000);
            rob.runCarouselRightTimeSpeed(0.25, 6000);
            rob.driveTrainEncoderMovement(speed, 60, Goal.movements.backward);
            rob.driveTrainEncoderMovement(speed, 20, Goal.movements.ccw);
            rob.cappingPivot.setPosition(0.65);
            rob.moveLinearSlideInches(1, location.getLinearSlideInches(), rob.cappingLinearSlide);
            rob.driveTrainEncoderMovement(speed, 20, Goal.movements.backward);
            rob.cappingClaw.setPosition(0.2);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 37, Goal.movements.left);
            rob.cappingPivot.setPosition(0.65);
            rob.moveLinearSlideInches(1, location.getLinearSlideInches(), rob.cappingLinearSlide);
            rob.driveTrainEncoderMovement(speed, 15, Goal.movements.backward);
            rob.cappingClaw.setPosition(0.2);
        }

        sleep(500);
        rob.driveTrainEncoderMovement(speed, 12, Goal.movements.forward);
        rob.moveLinearSlideInches(1, -location.getLinearSlideInches(), rob.cappingLinearSlide);
        rob.cappingClaw.setPosition(0.045);
        rob.cappingPivot.setPosition(0.2);
        rob.driveTrainEncoderMovement(speed, 20, Goal.movements.cw);
        rob.driveTrainEncoderMovement(speed, 26, Goal.movements.right);
        rob.driveTrainEncoderMovement(speed, 65, Goal.movements.backward);
        rob.driveTrainEncoderMovement(speed, 40, Goal.movements.left);
        rob.driveTrainEncoderMovement(speed, 25, Goal.movements.backward);
    }

    public void red(boolean doingCarousel) throws InterruptedException {
        rob.driveTrainEncoderMovement(speed, 7, Goal.movements.backward);
        determineLocation();

        if (doingCarousel) {
            rob.driveTrainEncoderMovement(speed, 40, Goal.movements.right);
            rob.driveTrainEncoderMovement(speed, 0.5, Goal.movements.forward);
            rob.runCarouselLeftTimeSpeed(-0.25, 6000);
            rob.runCarouselRightTimeSpeed(-0.25, 6000);
            rob.driveTrainEncoderMovement(speed, 75, Goal.movements.left);
            rob.cappingPivot.setPosition(0.65);
            rob.moveLinearSlideInches(1, location.getLinearSlideInches(), rob.cappingLinearSlide);
            rob.driveTrainEncoderMovement(speed, 16, Goal.movements.backward);
            rob.cappingClaw.setPosition(0.2);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 37, Goal.movements.right);
            rob.cappingPivot.setPosition(0.65);
            rob.moveLinearSlideInches(1, location.getLinearSlideInches(), rob.cappingLinearSlide);
            rob.driveTrainEncoderMovement(speed, 15, Goal.movements.backward);
            rob.cappingClaw.setPosition(0.2);
        }

        sleep(500);
        rob.driveTrainEncoderMovement(speed, 12, Goal.movements.forward);
        rob.moveLinearSlideInches(1, -location.getLinearSlideInches(), rob.cappingLinearSlide);
        rob.cappingClaw.setPosition(0.045);
        rob.cappingPivot.setPosition(0.2);
        rob.driveTrainEncoderMovement(speed, 20, Goal.movements.cw);
        rob.driveTrainEncoderMovement(speed, 19, Goal.movements.right);
        rob.driveTrainEncoderMovement(speed, 70, Goal.movements.forward);
        rob.driveTrainEncoderMovement(speed, 40, Goal.movements.left);
        rob.driveTrainEncoderMovement(speed, 25, Goal.movements.forward);
    }

    public void determineLocation() throws InterruptedException {
        if (rob.tfod != null) {
            sleep(1000);
            List<Recognition> finalRecognitions = null;
            for (int i = 0; i < 6; i++) {
                List<Recognition> recognitions = rob.tfod.getUpdatedRecognitions();
                if (recognitions.size() > 0) { finalRecognitions = recognitions; }
                rob.driveTrainEncoderMovement(speed, 1, Goal.movements.backward);
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






