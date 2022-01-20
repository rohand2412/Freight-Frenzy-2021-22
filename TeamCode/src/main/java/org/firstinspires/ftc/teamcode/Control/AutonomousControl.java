package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;

public abstract class AutonomousControl extends Central {
    static double speed = 0.5;
    public locations location;

    public void blue(boolean doingCarousel, boolean closeToCarousel, boolean parkShallowWarehouse) throws InterruptedException {
        initDetermineLocationSequence(!closeToCarousel);

        if (opModeIsActive()) {
            rob.driveTrainEncoderMovement(speed, 10.5, closeToCarousel ? Goal.movements.right : Goal.movements.left);

            if (doingCarousel) {
                if (closeToCarousel) {
                    rob.driveTrainEncoderMovement(speed, 10, Goal.movements.forward);
                    rob.driveTrainEncoderMovement(speed, 27, Goal.movements.right);
//                    rob.runCarouselsTimeSpeed(rob.carouselAuton, 4000);
                    rob.driveTrainEncoderMovement(speed, 65, Goal.movements.left);
//                    rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
                    if (location == locations.left || location == locations.middle) {
//                        rob.intakePivot.setPosition(0.7);
//                        rob.moveLinearSlideInches(1, location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
                    } else {
//                        rob.intakePivot.setPosition(0.6);
//                        rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
                    }
                    rob.driveTrainEncoderMovement(speed, 18, Goal.movements.forward);
//                    rob.intakeClaw.setPosition(0);
                }
            } else {
                rob.driveTrainEncoderMovement(speed, 20, Goal.movements.right);
//                rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
                if (location == locations.left || location == locations.middle) {
//                    rob.intakePivot.setPosition(0.7);
//                    rob.moveLinearSlideInches(1, location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
                    rob.driveTrainEncoderMovement(speed, 2, Goal.movements.forward);
                } else {
//                    rob.intakePivot.setPosition(0.6);
//                    rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
                }
                rob.driveTrainEncoderMovement(speed, 25, Goal.movements.forward);
//                rob.intakeClaw.setPosition(0);
            }

            sleep(500);
            rob.driveTrainEncoderMovement(speed, 12, Goal.movements.backward);
            if (location == locations.left) {
//                rob.intakePivot.setPosition(0);
//                rob.moveLinearSlideInches(1, -location.getLinearSlideInches(), rob.intakeLinearSlide);
            } else if (location == locations.middle) {
//                rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
//                rob.intakePivot.setPosition(0);
//                rob.moveLinearSlideInches(1, -location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
            } else {
//                rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
//                rob.intakePivot.setPosition(0);
//                rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
            }
            rob.turn(speed, -90);

            if (parkShallowWarehouse) {
                rob.driveTrainEncoderMovement(speed, 25, Goal.movements.left);
                rob.driveTrainEncoderMovement(speed, 75, Goal.movements.forward);
            } else {
                rob.driveTrainEncoderMovement(speed, 4, Goal.movements.right);
                rob.driveTrainEncoderMovement(1, 100, Goal.movements.forward);
            }
        }
    }

    public void red(boolean doingCarousel, boolean closeToCarousel, boolean parkShallowWarehouse) throws InterruptedException {
        initDetermineLocationSequence(closeToCarousel);

        if (opModeIsActive()) {
            rob.driveTrainEncoderMovement(speed, 10.5, closeToCarousel ? Goal.movements.left : Goal.movements.right);

            if (doingCarousel) {
                if (closeToCarousel) {
                    rob.driveTrainEncoderMovement(speed, 10, Goal.movements.forward);
                    rob.driveTrainEncoderMovement(speed, 47.5, Goal.movements.left);
//                    rob.runCarouselsTimeSpeed(rob.carouselAuton, 4000);
                    rob.driveTrainEncoderMovement(speed, 66, Goal.movements.right);
//                    rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
                    if (location == locations.left || location == locations.middle) {
//                        rob.intakePivot.setPosition(0.7);
//                        rob.moveLinearSlideInches(1, location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
                    } else {
//                        rob.intakePivot.setPosition(0.6);
//                        rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
                    }
                    rob.driveTrainEncoderMovement(speed, 17, Goal.movements.forward);
//                    rob.intakeClaw.setPosition(0);
                }
            } else {
                rob.driveTrainEncoderMovement(speed, 40, Goal.movements.left);
//                rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
                if (location == locations.left || location == locations.middle) {
//                    rob.intakePivot.setPosition(0.7);
//                    rob.moveLinearSlideInches(1, location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
                    rob.driveTrainEncoderMovement(speed, 4, Goal.movements.forward);
                } else {
//                    rob.intakePivot.setPosition(0.6);
//                    rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
                }
                rob.driveTrainEncoderMovement(speed, 25, Goal.movements.forward);
//                rob.intakeClaw.setPosition(0);
            }

            sleep(500);
            rob.driveTrainEncoderMovement(speed, 12, Goal.movements.backward);
            if (location == locations.left) {
//                rob.intakePivot.setPosition(0);
//                rob.moveLinearSlideInches(1, -location.getLinearSlideInches(), rob.intakeLinearSlide);
            } else if (location == locations.middle) {
//                rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
//                rob.intakePivot.setPosition(0);
//                rob.moveLinearSlideInches(1, -location.getLinearSlideInches() - 5, rob.intakeLinearSlide);
            } else {
//                rob.moveLinearSlideInches(1, 5, rob.intakeLinearSlide);
//                rob.intakePivot.setPosition(0);
//                rob.moveLinearSlideInches(1, -5, rob.intakeLinearSlide);
            }
            rob.turn(speed, 90);

            if (parkShallowWarehouse) {
                rob.driveTrainEncoderMovement(speed, 25, Goal.movements.right);
                rob.driveTrainEncoderMovement(speed, 75, Goal.movements.forward);
            } else {
                rob.driveTrainEncoderMovement(speed, 4, Goal.movements.left);
                rob.driveTrainEncoderMovement(1, 100, Goal.movements.forward);
            }
        }
    }

    public void initDetermineLocationSequence(boolean right) throws InterruptedException {
        while(!opModeIsActive() && !isStopRequested()) {
            if (waitWithStopCheck(100)) break;
            determineLocation(right);
        }

        if (!isStopRequested()) {
            telemetry.addLine("Final Location: " + (location != null ? location.toString() : "null"));
            telemetry.update();
        }
        waitAndStart();
    }

    public boolean waitWithStopCheck(long milliseconds) {
        boolean stopRequested = false;
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < milliseconds) {
            if (isStopRequested()) {
                stopRequested = true;
                break;
            }
        }
        return stopRequested;
    }

    public void determineLocation(boolean right) {
        if (rob.tfod != null) {
            List<Recognition> recognitions = rob.tfod.getUpdatedRecognitions();

            if (recognitions == null) {
                location = right ? locations.left : locations.right;
            }
            else if (recognitions.size() == 0) {
                location = right ? locations.left : locations.right;
            }
            else {
                if (countByLabelAndValid(recognitions, "Marker") == 1) {
                    int markerIndex = indexOfByLabel(recognitions, "Marker");
                    if ((recognitions.get(markerIndex).getLeft() + recognitions.get(markerIndex).getRight())/2 > 320) {
                        location = right ? locations.middle : locations.left;
                    }
                    else {
                        location = right ? locations.right : locations.middle;
                    }
                }
                else {
                    location = right ? locations.left : locations.right;
                }
            }

            telemetry.addLine(location != null ? location.toString() : "null");
            telemetry.addLine(recognitions != null ? recognitions.toString() : "null");
            telemetry.update();
        }
    }

    public boolean validRecognition(Recognition recognition) {
        return (recognition.getWidth() * recognition.getHeight()) < (320.0 * 640.0 / 8.0);
    }

    public int indexOfByLabel(List<Recognition> recognitions, String label) {
        for (int i = 0; i < recognitions.size(); i++) {
            if (recognitions.get(i).getLabel().equals(label)) return i;
        }
        return -1;
    }

    public int countByLabelAndValid(List<Recognition> recognitions, String label) {
        int count = 0;
        for (int i = 0; i < recognitions.size(); i++) {
            if (recognitions.get(i).getLabel().equals(label) && validRecognition(recognitions.get(i))) count++;
        }
        return count;
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






