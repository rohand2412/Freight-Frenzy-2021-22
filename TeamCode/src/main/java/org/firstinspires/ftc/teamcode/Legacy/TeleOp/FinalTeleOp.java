package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.teamcode.Control.Constants.BUCKET_PRELOAD_DEGREES;
import static org.firstinspires.ftc.teamcode.Control.Constants.DEAD_ZONE_SIZE;

@TeleOp(name="FinalTeleOp", group = "basic")
public class FinalTeleOp extends TeleOpControl
{
    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.teleop);

        boolean lifted = false;
        boolean crawlMode = false;
        double intakeSpeed = 0.5;
        double crawlModeSpeed = 0.2;
        boolean driveTrainRight = false;
        boolean driveTrainForward = true;
        double pivotDeg = 0;
        double bucketDeg = 0;
        double liftDeg = BUCKET_PRELOAD_DEGREES;
        double carouselSpeed = 0.6;
        Goal.ConditionalFunction lift = () -> !(validStick(xAxis1, yAxis1) || validStick(xAxis2, yAxis2));

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
                else if (gamepad1.right_stick_button) {
                    rob.driveTrainMovement(rt/crawlModeSpeed, Goal.movements.cw);
                }
                else if (gamepad1.left_stick_button) {
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
                else if (gamepad1.right_stick_button) {
                    rob.driveTrainMovement(rt, Goal.movements.cw);
                }
                else if (gamepad1.left_stick_button) {
                    rob.driveTrainMovement(lt, Goal.movements.ccw);
                }
                else {
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
                sleep(200);
            }

            if (rt2 > DEAD_ZONE_SIZE) {
//                rob.carousel.setPower(rt2 > carouselSpeed ? carouselSpeed : rt);
            }
            else if (lt2 > DEAD_ZONE_SIZE) {
//                rob.carousel.setPower(-(lt2 > carouselSpeed ? carouselSpeed : lt));
            }
            else {
//                rob.carousel.setPower(0);
            }

            if (validStick(xAxis4, yAxis4)) {
                rob.pivotCrane(rob.craneSpeed, pivotDeg + xAxis4 * 30);
                pivotDeg += xAxis4 * 30;
            }

//            else if (f(0)) {
//                rob.bucketSetDegree(--bucketDeg);
//            }
//            else if (f(2)) {
//                rob.bucketSetDegree(++bucketDeg);
//            }
//            else if (f(4)) {
//                rob.liftCraneHoldBucket(rob.craneSpeed, liftDeg + 5);
//                liftDeg += 5;
//            }
//            else if (f(6)) {
//                rob.liftCraneHoldBucket(rob.craneSpeed, liftDeg - 5);
//                liftDeg -= 5;
//            }

            if (!lifted && rb2) {
                rob.runIntakeSpeed(-0.1);
                rob.bucketMoveDegree(0, 10, lift);
                rob.runIntakeSpeed(0);
                rob.liftCraneHoldBucket(rob.craneSpeed, 130, lift);
                rob.bucketMoveDegree(190, 10, lift);
                if (lift.check()) lifted = true;
            }

            if (lifted && lb2) {
                rob.bucketSetDegree(0);
                rob.liftCraneHoldBucket(rob.craneSpeed, BUCKET_PRELOAD_DEGREES, lift);
                rob.runIntakeSpeed(0.1);
                rob.bucketSetDegree(120);
                rob.runIntakeSpeed(0);
                if (lift.check()) lifted = false;
            }

            if (gamepad2.x && lifted) rob.bucketMoveDegree(257, 10);
            if (gamepad2.b && lifted) rob.bucketSetDegree(190);

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