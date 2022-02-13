//package org.firstinspires.ftc.teamcode.TeleOps;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Control.Robot;
//import org.firstinspires.ftc.teamcode.Control._TeleOp;
//
//@TeleOp(group="TeleOp")
//public class FinalTeleOp extends _TeleOp {
//
//    @Override
//    public void init() {
//        Robot.setup(hardwareMap, telemetry, Robot.SetupType.TeleOp);
//        Robot.setCranePreset(Robot.CRANE_COLLECTION);
//    }
//
//    @Override
//    public void init_loop() {
//        Robot.update();
//    }
//
//    @Override
//    public void loop() {
//
//    }
//}

package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Robot;
import org.firstinspires.ftc.teamcode.Control._TeleOp;

@TeleOp(group="TeleOp")
public class FinalTeleOp extends _TeleOp {

    @Override
    public void init() {
        Robot.setup(hardwareMap, telemetry, Robot.SetupType.TeleOp);
        Robot.setFieldSide(Robot.FieldSide.RED);
        Robot.setCranePreset(Robot.CRANE_COLLECTION_DROP);
    }

    @Override
    public void init_loop() {
        Robot.update();
    }

    @Override
    public void loop() {
        Robot.update();
        telemetry.addLine("Lift: " + Robot.getCraneIMU().getRoll());
        telemetry.addLine("Pivot: " + Robot.getCraneIMU().getYaw());
        telemetry.addLine("Bucket" + Robot.getBucket().getDegree());

        if (gamepad1.a) {
            Robot.moveCraneToPreset(Robot.CRANE_TOP_LEVEL_HOLD, true);
        }
        else if (gamepad1.b) {
            Robot.getBucket().setDegree(Robot.CRANE_COLLECTION_HOLD.BUCKET_DEGREE);
            Robot.moveCraneToPreset(Robot.CRANE_COLLECTION_HOLD, false);
        }
        else if (gamepad1.x) {
            Robot.moveCraneToPreset(Robot.CRANE_SHARED_LEVEL_HOLD, true);
        }
        else if (gamepad1.y) {
            Robot.getBucket().setDegree(Robot.CRANE_COLLECTION_HOLD.BUCKET_DEGREE);
            Robot.moveCraneToPreset(Robot.CRANE_COLLECTION_HOLD, false);
        }

        double right_stick_y = -gamepad1.right_stick_y;
        double right_stick_x = gamepad1.right_stick_x;
        double atanResult = Math.atan2(right_stick_y, right_stick_x);
        double toDegrees = Math.toDegrees(atanResult);
        double joyStickAngle = (toDegrees + 360) % 360;
        double posAng = Robot.getIMU().getYaw();
        double speed = Math.hypot(right_stick_x, right_stick_y);

        double left_stick_y = -gamepad1.left_stick_y;
        double left_stick_x = gamepad1.left_stick_x;
        double atanResultCrawl = Math.atan2(left_stick_y, left_stick_x);
        double toDegreesCrawl = Math.toDegrees(atanResultCrawl);
        double joyStickAngleCrawl = (toDegreesCrawl + 360) % 360;
        double posAngCrawl = Robot.getIMU().getYaw();
        double speedCrawl = Math.hypot(right_stick_x, right_stick_y)/2.5;
        if(gamepad1.right_stick_x!=0 || gamepad1.right_stick_y!=0){

            Robot.getDrivetrain().runSpeedAngle(speed,-posAng + (joyStickAngle - 90),0);
        }
        else if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0){
            Robot.getDrivetrain().runSpeedAngle(speedCrawl,-posAngCrawl + (joyStickAngleCrawl - 90),0);
        }

        //carousel cw
        if(gamepad2.right_trigger!=0){
            Robot.getCarousel().runSpeed(0.5);
        }
        //carouselccw
        else if(gamepad2.left_trigger!=0){
            Robot.getCarousel().runSpeed(-0.5);
        }
        //pivot ccw
        else if(gamepad2.dpad_left){
            Robot.setCranePivotDegree(Robot.getCraneIMU().getYaw()-10);
        }
        //pivot cw
        else if(gamepad2.dpad_right){
            Robot.setCranePivotDegree(Robot.getCraneIMU().getYaw()+10);
        }
        //arm lift up
        else if(gamepad2.dpad_up){
            Robot.setCraneLiftDegree(Robot.getCraneIMU().getPitch()+.1);
        }
        //arm lift down
        else if(gamepad2.dpad_down){
            Robot.setCraneLiftDegree(Robot.getCraneIMU().getPitch()-.1);
        }
        //bucket up
        else if(gamepad2.right_bumper){
            Robot.getBucket().setDegree(Robot.getBucket().getDegree()+1);
        }
        //bucket down
        else if(gamepad2.left_bumper){
            Robot.getBucket().setDegree(Robot.getBucket().getDegree()-1);
        }
        else if(gamepad2.y){
            //if the bucket is in the intake
            if(Robot.getCraneIMU().getRoll()>=-68
                    && Robot.getCraneIMU().getRoll()<=-62
                    && Robot.getBucket().getDegree() <= 135
                    && Robot.getBucket().getDegree() >= 125) {

                Robot.getIntake().setTypicalSpeed(-0.5);
                Robot.getIntake().runTime(3000);
                Robot.getBucket().setSlowDegree(40,1000);
            }
            //if bucket is outside go to intake
            else{
                Robot.getIntake().runSpeed(0.5);
                Robot.setCraneLiftDegree(-65.5);
                if(Robot.getCraneIMU().getRoll()<-62 && Robot.getCraneIMU().getRoll()>-68){
                    Robot.getBucket().setSlowDegree(130,1000);
                }

            }
        }
        //reset imu
        //help
        else if(gamepad2.b){
           // Robot.getIMU().reset();
        }
        //drop
        //help with setSlowDegree
        else if(gamepad2.a){
            //Robot.getBucket().setSlowDegree(/*insert degree*/,1.55);
        }
        else if(gamepad2.x){
            Robot.getDrivetrain().stop();
        }
        //help
        //moving collection to preset or preset to preset
        //capping pickup
        else if(gamepad2.left_stick_button){

        }
        //help
        //preset to preset
        //capping drop off
        else if(gamepad2.right_stick_button){
            if(Robot.getCraneIMU().getPitch()>=-5
                    && Robot.getCraneIMU().getPitch()<=5){

            }
            else{

            }
        }
    }
}
