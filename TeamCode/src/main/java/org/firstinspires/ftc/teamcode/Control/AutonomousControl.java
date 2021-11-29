package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import static org.firstinspires.ftc.teamcode.Control.Constants.LABEL_FIRST_ELEMENT;
import static org.firstinspires.ftc.teamcode.Control.Constants.LABEL_SECOND_ELEMENT;

public abstract class AutonomousControl extends Central {
    static double speed = 0.5;
    locations location;

    public void blue(boolean doingCarousel, boolean closeToCarousel, boolean parkShallowWarehouse) throws InterruptedException {
        rob.driveTrainEncoderMovement(speed, 10.5, closeToCarousel ? Goal.movements.right : Goal.movements.left);
        determineLocation(!closeToCarousel);
        sleep(1000);

        if (doingCarousel) {
            if (closeToCarousel) {
                rob.driveTrainEncoderMovement(speed, 10, Goal.movements.forward);
                rob.driveTrainEncoderMovement(speed, 27, Goal.movements.right);
                rob.runCarouselsTimeSpeed(rob.carouselAuton, 4000);
                rob.driveTrainEncoderMovement(speed, 65, Goal.movements.left);
                rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
                if (location == locations.left || location == locations.middle) {
                    rob.intakePivot.setPosition(0.7);
                    rob.moveLinearSlideInches(1, location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
                }
                else {
                    rob.intakePivot.setPosition(0.6);
                    rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
                }
                rob.driveTrainEncoderMovement(speed, 18, Goal.movements.forward);
                rob.intakeClaw.setPosition(0);
            }
        }
        else {
            rob.driveTrainEncoderMovement(speed, 20, Goal.movements.right);
            rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
            if (location == locations.left || location == locations.middle) {
                rob.intakePivot.setPosition(0.7);
                rob.moveLinearSlideInches(1, location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
                rob.driveTrainEncoderMovement(speed, 2, Goal.movements.forward);
            }
            else {
                rob.intakePivot.setPosition(0.6);
                rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
            }
            rob.driveTrainEncoderMovement(speed, 25, Goal.movements.forward);
            rob.intakeClaw.setPosition(0);
        }

        sleep(500);
        rob.driveTrainEncoderMovement(speed, 12, Goal.movements.backward);
        if (location == locations.left) {
            rob.intakePivot.setPosition(0);
            rob.moveLinearSlideInches(1, -location.getLinearSlideInches(), rob.intakeLinearSlide);
        }
        else if (location == locations.middle) {
            rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
            rob.intakePivot.setPosition(0);
            rob.moveLinearSlideInches(1, -location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
        }
        else {
            rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
            rob.intakePivot.setPosition(0);
            rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
        }
        rob.turn(speed, -90);

        if (parkShallowWarehouse) {
            rob.driveTrainEncoderMovement(speed, 25, Goal.movements.left);
            rob.driveTrainEncoderMovement(speed, 75, Goal.movements.forward);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 4, Goal.movements.right);
            rob.driveTrainEncoderMovement(1, 100, Goal.movements.forward);
        }
    }

    public void red(boolean doingCarousel, boolean closeToCarousel, boolean parkShallowWarehouse) throws InterruptedException {
        rob.driveTrainEncoderMovement(speed, 10.5, closeToCarousel ? Goal.movements.left : Goal.movements.right);
        determineLocation(closeToCarousel);
        sleep(1000);

        if (doingCarousel) {
            if (closeToCarousel) {
                rob.driveTrainEncoderMovement(speed, 10, Goal.movements.forward);
                rob.driveTrainEncoderMovement(speed, 47.5, Goal.movements.left);
                rob.runCarouselsTimeSpeed(rob.carouselAuton, 4000);
                rob.driveTrainEncoderMovement(speed, 66, Goal.movements.right);
                rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
                if (location == locations.left || location == locations.middle) {
                    rob.intakePivot.setPosition(0.7);
                    rob.moveLinearSlideInches(1, location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
                }
                else {
                    rob.intakePivot.setPosition(0.6);
                    rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
                }
                rob.driveTrainEncoderMovement(speed, 17, Goal.movements.forward);
                rob.intakeClaw.setPosition(0);
            }
        }
        else {
            rob.driveTrainEncoderMovement(speed, 40, Goal.movements.left);
            rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
            if (location == locations.left || location == locations.middle) {
                rob.intakePivot.setPosition(0.7);
                rob.moveLinearSlideInches(1, location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
                rob.driveTrainEncoderMovement(speed, 4, Goal.movements.forward);
            }
            else {
                rob.intakePivot.setPosition(0.6);
                rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
            }
            rob.driveTrainEncoderMovement(speed, 25, Goal.movements.forward);
            rob.intakeClaw.setPosition(0);
        }

        sleep(500);
        rob.driveTrainEncoderMovement(speed, 12, Goal.movements.backward);
        if (location == locations.left) {
            rob.intakePivot.setPosition(0);
            rob.moveLinearSlideInches(1, -location.getLinearSlideInches(), rob.intakeLinearSlide);
        }
        else if (location == locations.middle) {
            rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
            rob.intakePivot.setPosition(0);
            rob.moveLinearSlideInches(1, -location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
        }
        else {
            rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
            rob.intakePivot.setPosition(0);
            rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
        }
        rob.turn(speed, 90);

        if (parkShallowWarehouse) {
            rob.driveTrainEncoderMovement(speed, 25, Goal.movements.right);
            rob.driveTrainEncoderMovement(speed, 75, Goal.movements.forward);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 4, Goal.movements.left);
            rob.driveTrainEncoderMovement(1, 100, Goal.movements.forward);
        }
    }

    public void determineLocation(boolean right) throws InterruptedException {
        if (rob.tfod != null) {
            sleep(1000);
            List<Recognition> finalRecognitions = null;
            for (int i = 0; i < 10; i++) {
                List<Recognition> recognitions = rob.tfod.getUpdatedRecognitions();
                if (recognitions != null) if (recognitions.size() > 0) { finalRecognitions = recognitions; }
            }

            if (right) {
                if (finalRecognitions == null) location = locations.left;
                else if (finalRecognitions.size() == 0) location = locations.left;
                else if (finalRecognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT))
                    location = locations.middle;
                else if (finalRecognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT))
                    location = locations.right;
            }
            else {
                if (finalRecognitions == null) location = locations.right;
                else if (finalRecognitions.size() == 0) location = locations.right;
                else if (finalRecognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT))
                    location = locations.left;
                else if (finalRecognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT))
                    location = locations.middle;
            }

            telemetry.addLine(location.toString());
            telemetry.update();
        }
    }

    public enum locations {
        left("left", 7.5),
        middle("middle", 1),
        right("right", -4.75);

        private final String name;
        private final double linearSlideInches;

        locations(String name, double linearSlideInches) { this.name = name; this.linearSlideInches = linearSlideInches; }

        public String toString() { return this.name; }

        public double getLinearSlideInches() { return this.linearSlideInches; }
    }
}






