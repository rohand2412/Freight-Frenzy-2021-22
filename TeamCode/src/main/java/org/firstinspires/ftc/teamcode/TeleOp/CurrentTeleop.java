package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_DEGREE_GOBILDA_30_RPM;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_INCH_LINEAR_SLIDE_MOTOR;
import static org.firstinspires.ftc.teamcode.Control.Constants.DEAD_ZONE_SIZE;

@TeleOp(name="CurrentTeleop", group = "basic")
public class CurrentTeleop extends TeleOpControl
{
    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.teleop);
        rob.intakeClaw.setPosition(0);

        boolean intakeUp = false;
        boolean holdingObject = false;
        boolean intakeClawOpen = true;
        boolean crawlMode = false;
        double intakeSpeed = 0.5;
        double intakePivotPosition = 0.7;
        double speedScalar = 0.75;

        waitForStart();

        while (opModeIsActive()) {
            standardGamepadData();

            if (crawlMode) {
                if (g(0)) {
                    rob.driveTrainMovement(0.2, Goal.movements.forward);
                } else if (g(2)) {
                    rob.driveTrainMovement(0.2, Goal.movements.backward);
                } else if (g(3)) {
                    rob.driveTrainMovement(0.2, Goal.movements.right);
                } else if (g(1)) {
                    rob.driveTrainMovement(0.2, Goal.movements.left);
                } else if (g(4)) {
                    rob.driveTrainMovement(0.2, Goal.movements.br);
                } else if (g(5)) {
                    rob.driveTrainMovement(0.2, Goal.movements.bl);
                } else if (g(6)) {
                    rob.driveTrainMovement(0.2, Goal.movements.tl);
                } else if (g(7)) {
                    rob.driveTrainMovement(0.2, Goal.movements.tr);
                } else if (g(8)) {
                    rob.driveTrainMovement(0.2, Goal.movements.ccw);
                } else if (g(9)) {
                    rob.driveTrainMovement(0.2, Goal.movements.cw);
                } else {
                    rob.stopDrivetrain();
                }
            }
            else {
                if (g(0)) {
                    rob.driveTrainMovement(fb * speedScalar, Goal.movements.forward);
                } else if (g(2)) {
                    rob.driveTrainMovement(fb * speedScalar, Goal.movements.backward);
                } else if (g(3)) {
                    rob.driveTrainMovement(rl * speedScalar, Goal.movements.right);
                } else if (g(1)) {
                    rob.driveTrainMovement(rl * speedScalar, Goal.movements.left);
                } else if (g(4)) {
                    rob.driveTrainMovement(diagonalSpeed * speedScalar, Goal.movements.br);
                } else if (g(5)) {
                    rob.driveTrainMovement(diagonalSpeed * speedScalar, Goal.movements.bl);
                } else if (g(6)) {
                    rob.driveTrainMovement(diagonalSpeed * speedScalar, Goal.movements.tl);
                } else if (g(7)) {
                    rob.driveTrainMovement(diagonalSpeed * speedScalar, Goal.movements.tr);
                } else if (g(8)) {
                    rob.driveTrainMovement(1 * speedScalar, Goal.movements.ccw);
                } else if (g(9)) {
                    rob.driveTrainMovement(1 * speedScalar, Goal.movements.cw);
                } else {
                    rob.stopDrivetrain();
                }
            }

            if (rt > DEAD_ZONE_SIZE) {
                rob.runIntakeSpeed(-intakeSpeed);
            }
            else if (lt > DEAD_ZONE_SIZE) {
                rob.runIntakeSpeed(intakeSpeed);
            }
            else if (gamepad1.y) {
                rob.runIntakeSpeed(0);
            }

            if (gamepad1.b) {
                crawlMode = !crawlMode;
            }

            if (rt2 > 0.05) {
                rob.runCarouselsSpeed(rt2 > rob.carouselTele ? rob.carouselTele : rt2);
            }
            else if (lt2 > 0.05) {
                rob.runCarouselsSpeed(-(lt2 > rob.carouselTele ? rob.carouselTele : lt2));
            }
            else {
                rob.runCarouselsSpeed(0);
            }

            if (rb2) {
                int target = rob.intakeLinearSlide.getTargetPosition();
                rob.moveLinearSlideInches(1, 1, rob.intakeLinearSlide);
                rob.intakeLinearSlide.setTargetPosition(target);
            }
            else if (lb2) {
                int target = rob.intakeLinearSlide.getTargetPosition();
                rob.moveLinearSlideInches(1, -1, rob.intakeLinearSlide);
                rob.intakeLinearSlide.setTargetPosition(target);
            }

            if (gamepad1.a && !intakeUp && !holdingObject) {
                rob.intakeClaw.setPosition(1);
                rob.runIntakeSpeed(0);
                rob.moveLinearSlideInches(1, 1, rob.intakeLinearSlide);
                intakeClawOpen = false;
                holdingObject = true;
            }
            else if (gamepad1.a && !intakeUp && holdingObject) {
                rob.moveLinearSlideInches(1, -1, rob.intakeLinearSlide);
                rob.intakeClaw.setPosition(0);
                intakeClawOpen = true;
                holdingObject = false;
            }
            else if (gamepad1.a && intakeUp && holdingObject) {
                rob.intakeClaw.setPosition(0);
                rob.driveTrainEncoderMovement(0.5, 12, Goal.movements.backward);
                rob.intakePivot.setPosition(0);
                rob.moveLinearSlideInches(1, -8, rob.intakeLinearSlide);
                crawlMode = false;
                intakeUp = false;
                holdingObject = false;
                intakeClawOpen = true;
            }

            if (gamepad1.x && !intakeUp && holdingObject) {
                rob.moveLinearSlideInches(1, 7, rob.intakeLinearSlide);
                rob.intakePivot.setPosition(intakePivotPosition);
                crawlMode = true;
                intakeUp = true;
            }

            if (gamepad2.dpad_up) {
                if (intakePivotPosition < 0.9) intakePivotPosition += 0.001;
                rob.intakePivot.setPosition(intakePivotPosition);
            }
            else if (gamepad2.dpad_down) {
                if (intakePivotPosition > 0) intakePivotPosition -= 0.001;
                rob.intakePivot.setPosition(intakePivotPosition);
            }

            if (gamepad2.b) {
                if (intakeClawOpen) {
                    rob.intakeClaw.setPosition(1);
                    intakeClawOpen = false;
                }
                else {
                    rob.intakeClaw.setPosition(0);
                    intakeClawOpen = true;
                }
            }

            if (gamepad2.y) {
                rob.stopDrivetrain();
                sleep(250);
                rob.runIntakeSpeed(0);
                sleep(250);
//                rob.moveLinearSlide(0);
                sleep(250);
                rob.runCarouselsSpeed(0);
                sleep(250);
            }
        }
    }
}