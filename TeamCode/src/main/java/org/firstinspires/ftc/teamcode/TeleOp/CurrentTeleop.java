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

        boolean intakeToggle = true; //true is down, false is up
        boolean alreadyLowered = false;
        boolean crawlMode = false;
        boolean linearSlideUp = false;
        boolean needToRaise = false;
        boolean drivetrainInUse;
        double carouselSpeed = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            standardGamepadData();

            drivetrainInUse = false;
            if (crawlMode) {
                if (g(0)) {
                    rob.driveTrainMovement(0.2, Goal.movements.forward);
                    drivetrainInUse = true;
                } else if (g(2)) {
                    rob.driveTrainMovement(0.2, Goal.movements.backward);
                    drivetrainInUse = true;
                } else if (g(3)) {
                    rob.driveTrainMovement(0.2, Goal.movements.right);
                    drivetrainInUse = true;
                } else if (g(1)) {
                    rob.driveTrainMovement(0.2, Goal.movements.left);
                    drivetrainInUse = true;
                } else if (g(4)) {
                    rob.driveTrainMovement(0.2, Goal.movements.br);
                    drivetrainInUse = true;
                } else if (g(5)) {
                    rob.driveTrainMovement(0.2, Goal.movements.bl);
                    drivetrainInUse = true;
                } else if (g(6)) {
                    rob.driveTrainMovement(0.2, Goal.movements.tl);
                    drivetrainInUse = true;
                } else if (g(7)) {
                    rob.driveTrainMovement(0.2, Goal.movements.tr);
                    drivetrainInUse = true;
                } else if (g(8)) {
                    rob.driveTrainMovement(0.2, Goal.movements.ccw);
                    drivetrainInUse = true;
                } else if (g(9)) {
                    rob.driveTrainMovement(0.2, Goal.movements.cw);
                    drivetrainInUse = true;
                } else {
                    rob.stopDrivetrain();
                }
            }
            else {
                if (g(0)) {
                    rob.driveTrainMovement(fb, Goal.movements.forward);
                    drivetrainInUse = true;
                } else if (g(2)) {
                    rob.driveTrainMovement(fb, Goal.movements.backward);
                    drivetrainInUse = true;
                } else if (g(3)) {
                    rob.driveTrainMovement(rl, Goal.movements.right);
                    drivetrainInUse = true;
                } else if (g(1)) {
                    rob.driveTrainMovement(rl, Goal.movements.left);
                    drivetrainInUse = true;
                } else if (g(4)) {
                    rob.driveTrainMovement(diagonalSpeed, Goal.movements.br);
                    drivetrainInUse = true;
                } else if (g(5)) {
                    rob.driveTrainMovement(diagonalSpeed, Goal.movements.bl);
                    drivetrainInUse = true;
                } else if (g(6)) {
                    rob.driveTrainMovement(diagonalSpeed, Goal.movements.tl);
                    drivetrainInUse = true;
                } else if (g(7)) {
                    rob.driveTrainMovement(diagonalSpeed, Goal.movements.tr);
                    drivetrainInUse = true;
                } else if (g(8)) {
                    rob.driveTrainMovement(1, Goal.movements.ccw);
                    drivetrainInUse = true;
                } else if (g(9)) {
                    rob.driveTrainMovement(1, Goal.movements.cw);
                    drivetrainInUse = true;
                } else {
                    rob.stopDrivetrain();
                }
            }



            if (rt > DEAD_ZONE_SIZE) {
                if (intakeToggle) rob.runIntakeSpeed(-1);
            }
            else if (gamepad1.y) {
                rob.runIntakeSpeed(0);
            }

            if (lt > DEAD_ZONE_SIZE) {
                if (intakeToggle) rob.runIntakeSpeed(0.75);
            }
            else if (rob.intake.getPower() > 0 && !needToRaise) {
                rob.runIntakeSpeed(0);
            }

            if (rb) {
                if (rob.linearSlide.getCurrentPosition() < 1665) {
                    rob.moveLinearSlide(1);
                }
            }
            else if (lb) {
                if (rob.linearSlide.getCurrentPosition() > 10) {
                    rob.moveLinearSlide(-1);
                }
            }
            else {
                rob.moveLinearSlide(0);
            }



            if (gamepad1.b) {
                crawlMode = !crawlMode;
            }

            if (gamepad1.dpad_right && rob.claw.getPosition() < 0.2) {
                rob.rotate.setPosition(-0.001 + rob.rotate.getPosition());
            }
            else if (gamepad1.dpad_left) {
                rob.rotate.setPosition(0.001 + rob.rotate.getPosition());
            }

            if (gamepad1.dpad_up && rob.rotate.getPosition() >= 0.5) {
                rob.claw.setPosition(0.2);
            }
            else if (gamepad1.dpad_down) {
                rob.claw.setPosition(0);
            }

            if (rb2) {
                carouselSpeed = 0.55;
            }
            else if (lb2) {
                carouselSpeed = 0.5;
            }

            if (rt2 > DEAD_ZONE_SIZE) {
                rob.runCarouselSpeed(carouselSpeed);
            }
            else if (lt2 > DEAD_ZONE_SIZE) {
                rob.runCarouselSpeed(-carouselSpeed);
            }
            else {
                rob.runCarouselSpeed(0);
            }


            if (gamepad2.dpad_right) {
                rob.claw.setPosition(0);
            }

            if (gamepad2.x) {
                if (linearSlideUp) {
                    rob.moveSingleMotorUnits(-1, -1600.0, 1, rob.linearSlide);
                }
                else {
                    rob.moveSingleMotorUnits(1, 1600.0, 1, rob.linearSlide);
                }
                linearSlideUp = !linearSlideUp;
            }



            if (gamepad2.y) {
                rob.stopDrivetrain();
                sleep(250);
                rob.runIntakeSpeed(0);
                sleep(250);
                rob.moveLinearSlide(0);
                sleep(250);
                rob.runCarouselSpeed(0);
                sleep(250);
            }
        }
    }
}