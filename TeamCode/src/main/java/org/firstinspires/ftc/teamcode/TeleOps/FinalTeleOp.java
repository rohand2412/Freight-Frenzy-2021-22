package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Robot;
import org.firstinspires.ftc.teamcode.Control._TeleOp;
import org.firstinspires.ftc.teamcode.Drivers._Drivetrain;

@TeleOp(group="TeleOp")
public class FinalTeleOp extends _TeleOp {
    @Override
    public void init() {
        Robot.setup(hardwareMap,telemetry, Robot.SetupType.Everything);
    }

    @Override
    public void loop() {
        Robot.update();
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
            Robot.getDrivetrain().runSpeed(0.8, _Drivetrain.Movements.ccw);
        }
        //pivot cw
        else if(gamepad2.dpad_right){
            Robot.getDrivetrain().runSpeed(0.8,_Drivetrain.Movements.cw);
        }
        //arm lift up
        else if(gamepad2.dpad_up){
            Robot.setCraneLiftDegree(Robot.getCraneIMU().getPitch()+2);
        }
        //arm lift down
        else if(gamepad2.dpad_down){
            Robot.setCraneLiftDegree(Robot.getCraneIMU().getPitch()-2);
        }
        //bucket up
        else if(gamepad2.right_bumper){
            Robot.getBucket().setDegree(Robot.getBucket().getDegree()+2);
        }
        //bucket down
        else if(gamepad2.left_bumper){
            Robot.getBucket().setDegree(Robot.getBucket().getDegree()-2);
        }
        else if(gamepad2.y){
            //if the bucket is in the intake
            if(Robot.getCraneIMU().getPitch()>=-68
                    && Robot.getCraneIMU().getPitch()<=-62
                    && Robot.getBucket().getDegree()<= 135
                    && Robot.getBucket().getDegree()>=125){

                Robot.getIntake().setTypicalSpeed(-0.3);
                Robot.getIntake().runTime(3000);
                Robot.getBucket().setSlowDegree(40,10);
            }
            //if bucket is outside go to intake
            else{
                Robot.getIntake().runSpeed(.3);
                Robot.setCraneLiftDegree(-65.5);
                if(Robot.getCraneIMU().getRoll()<-62 && Robot.getCraneIMU().getRoll()>-68){
                    Robot.getBucket().setDegree(130);
                }
            }
        }
        //reset imu
        //help
        else if(gamepad2.b){
            Robot.getIMU().reset();
        }
        //drop
        //help with setSlowDegree
        else if(gamepad2.a){
            Robot.getBucket().setSlowDegree(/*insert degree*/,1.55);
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
