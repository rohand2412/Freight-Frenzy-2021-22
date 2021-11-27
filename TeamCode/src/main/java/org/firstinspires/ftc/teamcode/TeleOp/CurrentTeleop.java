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
        boolean intakeUp = false;
        boolean crawlMode = false;
        double carouselSpeed = 0.8;

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
                    rob.driveTrainMovement(fb, Goal.movements.forward);
                } else if (g(2)) {
                    rob.driveTrainMovement(fb, Goal.movements.backward);
                } else if (g(3)) {
                    rob.driveTrainMovement(rl, Goal.movements.right);
                } else if (g(1)) {
                    rob.driveTrainMovement(rl, Goal.movements.left);
                } else if (g(4)) {
                    rob.driveTrainMovement(diagonalSpeed, Goal.movements.br);
                } else if (g(5)) {
                    rob.driveTrainMovement(diagonalSpeed, Goal.movements.bl);
                } else if (g(6)) {
                    rob.driveTrainMovement(diagonalSpeed, Goal.movements.tl);
                } else if (g(7)) {
                    rob.driveTrainMovement(diagonalSpeed, Goal.movements.tr);
                } else if (g(8)) {
                    rob.driveTrainMovement(1, Goal.movements.ccw);
                } else if (g(9)) {
                    rob.driveTrainMovement(1, Goal.movements.cw);
                } else {
                    rob.stopDrivetrain();
                }
            }

            if (rt > DEAD_ZONE_SIZE) {
                rob.runIntakeSpeed(-0.75);
            }
            else if (lt > DEAD_ZONE_SIZE) {
                rob.runIntakeSpeed(0.75);
            }
            else if (gamepad1.y) {
                rob.runIntakeSpeed(0);
            }

            if (gamepad1.x) {
                crawlMode = !crawlMode;
            }

            if (rt2 > 0.05) {
                rob.runCarouselsSpeed(rt2 > carouselSpeed ? carouselSpeed : rt2);
            }
            else if (lt2 > 0.05) {
                rob.runCarouselsSpeed(-(lt2 > carouselSpeed ? carouselSpeed : lt2));
            }
            else {
                rob.runCarouselsSpeed(0);
            }

            if (rb2) {
                int target = rob.intakeLinearSlide.getTargetPosition();
                rob.moveLinearSlideInches(1, 0.5, rob.intakeLinearSlide);
                rob.intakeLinearSlide.setTargetPosition(target);
            }
            else if (lb2) {
                int target = rob.intakeLinearSlide.getTargetPosition();
                rob.moveLinearSlideInches(1, -0.5, rob.intakeLinearSlide);
                rob.intakeLinearSlide.setTargetPosition(target);
            }

            if (gamepad2.x) { carouselSpeed = 0.7; }
            if (gamepad2.a) { carouselSpeed = 0.6; }

            if (gamepad1.a && !intakeUp) {
                rob.intakeClaw.setPosition(1);
                rob.runIntakeSpeed(0);
                rob.moveLinearSlideInches(1, 8, rob.intakeLinearSlide);
                rob.intakePivot.setPosition(0.65);
                intakeUp = true;
            }
            else if (gamepad1.a && intakeUp) {
                rob.intakeClaw.setPosition(0);
                rob.driveTrainEncoderMovement(0.5, 6, Goal.movements.backward);
                rob.intakePivot.setPosition(0);
                rob.moveLinearSlideInches(1, -8, rob.intakeLinearSlide);
                crawlMode = false;
                intakeUp = false;
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