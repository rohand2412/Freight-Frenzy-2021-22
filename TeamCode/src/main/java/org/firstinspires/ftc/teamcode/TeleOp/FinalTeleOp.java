package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.teamcode.Control.Constants.DEAD_ZONE_SIZE;

@TeleOp(name="FinalTeleOp", group = "basic")
public class FinalTeleOp extends TeleOpControl
{
    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.teleop);
//        rob.intakeClaw.setPosition(0);

        boolean intakeUp = false;
        boolean holdingObject = false;
        boolean intakeClawOpen = true;
        boolean crawlMode = false;
        double intakeSpeed = 0.5;
        double intakePivotPosition = 0.7;
        double crawlModeSpeed = 0.2;
        boolean driveTrainRight = false;
        boolean driveTrainForward = true;

        waitForStart();

        while (opModeIsActive()) {
            standardGamepadData();

            if (crawlMode) {
                if (validStick(xAxis1, yAxis1)) {
                    rob.driveTrainMovementAngleRadians((Math.hypot(xAxis1, yAxis1)/Math.sqrt(2))/crawlModeSpeed, Goal.quadrantAtan(xAxis1, yAxis1), driveTrainForward ? Math.toRadians(0) : Math.toRadians(180));
                }
                else if (validStick(xAxis2, yAxis2)) {
                    rob.driveTrainMovementAngleRadians(Math.hypot(xAxis2, yAxis2)/Math.sqrt(2)/crawlModeSpeed, Goal.quadrantAtan(xAxis2, yAxis2), driveTrainRight ? Math.toRadians(-90) : Math.toRadians(90));
                }
                else if (rt > DEAD_ZONE_SIZE) {
                    rob.driveTrainMovement(rt/crawlModeSpeed, Goal.movements.cw);
                }
                else if (lt > DEAD_ZONE_SIZE) {
                    rob.driveTrainMovement(lt/crawlModeSpeed, Goal.movements.ccw);
                }
                else {
                    rob.stopDrivetrain();
                }
            }
            else {
                if (validStick(xAxis1, yAxis1)) {
                    rob.driveTrainMovementAngleRadians(Math.hypot(xAxis1, yAxis1)/Math.sqrt(2), Goal.quadrantAtan(xAxis1, yAxis1), driveTrainForward ? Math.toRadians(0) : Math.toRadians(180));
                }
                else if (validStick(xAxis2, yAxis2)) {
                    rob.driveTrainMovementAngleRadians(Math.hypot(xAxis2, yAxis2)/Math.sqrt(2), Goal.quadrantAtan(xAxis2, yAxis2), driveTrainRight ? Math.toRadians(-90) : Math.toRadians(90));
                }
                else if (rt > DEAD_ZONE_SIZE) {
                    rob.driveTrainMovement(rt, Goal.movements.cw);
                }
                else if (lt > DEAD_ZONE_SIZE) {
                    rob.driveTrainMovement(lt, Goal.movements.ccw);
                }
                else {
                    rob.stopDrivetrain();
                }
            }

            if (gamepad1.left_stick_button) driveTrainForward = !driveTrainForward;
            if (gamepad1.right_stick_button) driveTrainRight = !driveTrainRight;

            if (rb) {
                rob.runIntakeSpeed(-intakeSpeed);
            }
            else if (lb) {
                rob.runIntakeSpeed(intakeSpeed);
            }
            else if (gamepad1.y) {
                rob.runIntakeSpeed(0);
            }

            if (gamepad1.b) {
                crawlMode = !crawlMode;
            }

            if (rt2 > DEAD_ZONE_SIZE) {
//                rob.runCarouselsSpeed(rt2 > rob.carouselTele ? rob.carouselTele : rt2);
            }
            else if (lt2 > DEAD_ZONE_SIZE) {
//                rob.runCarouselsSpeed(-(lt2 > rob.carouselTele ? rob.carouselTele : lt2));
            }
            else {
//                rob.runCarouselsSpeed(0);
            }

            if (rb2) {
//                int target = rob.intakeLinearSlide.getTargetPosition();
//                rob.moveLinearSlideInches(1, 1, rob.intakeLinearSlide);
//                rob.intakeLinearSlide.setTargetPosition(target);
            }
            else if (lb2) {
//                int target = rob.intakeLinearSlide.getTargetPosition();
//                rob.moveLinearSlideInches(1, -1, rob.intakeLinearSlide);
//                rob.intakeLinearSlide.setTargetPosition(target);
            }

            if (gamepad1.a && !intakeUp && !holdingObject) {
//                rob.intakeClaw.setPosition(1);
                rob.runIntakeSpeed(0);
//                rob.moveLinearSlideInches(1, 1, rob.intakeLinearSlide);
                intakeClawOpen = false;
                holdingObject = true;
            }
            else if (gamepad1.a && !intakeUp && holdingObject) {
//                rob.moveLinearSlideInches(1, -1, rob.intakeLinearSlide);
//                rob.intakeClaw.setPosition(0);
                intakeClawOpen = true;
                holdingObject = false;
            }
            else if (gamepad1.a && intakeUp && holdingObject) {
//                rob.intakeClaw.setPosition(0);
                rob.driveTrainEncoderMovement(0.5, 12, Goal.movements.backward);
//                rob.intakePivot.setPosition(0);
//                rob.moveLinearSlideInches(1, -8, rob.intakeLinearSlide);
                crawlMode = false;
                intakeUp = false;
                holdingObject = false;
                intakeClawOpen = true;
            }

            if (gamepad1.x && !intakeUp && holdingObject) {
//                rob.moveLinearSlideInches(1, 7, rob.intakeLinearSlide);
//                rob.intakePivot.setPosition(intakePivotPosition);
                crawlMode = true;
                intakeUp = true;
            }

            if (gamepad2.dpad_up) {
                if (intakePivotPosition < 0.9) intakePivotPosition += 0.001;
//                rob.intakePivot.setPosition(intakePivotPosition);
            }
            else if (gamepad2.dpad_down) {
                if (intakePivotPosition > 0) intakePivotPosition -= 0.001;
//                rob.intakePivot.setPosition(intakePivotPosition);
            }

            if (gamepad2.b) {
                if (intakeClawOpen) {
//                    rob.intakeClaw.setPosition(1);
                    intakeClawOpen = false;
                }
                else {
//                    rob.intakeClaw.setPosition(0);
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
//                rob.runCarouselsSpeed(0);
                sleep(250);
            }
        }
    }
}