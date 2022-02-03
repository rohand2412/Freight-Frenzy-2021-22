package org.firstinspires.ftc.teamcode.DriverTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Control.Robot;
import org.firstinspires.ftc.teamcode.Control._Autonomous;
import org.firstinspires.ftc.teamcode.Drivers._Motor;
import org.firstinspires.ftc.teamcode.Drivers._Servo;

@Autonomous(name="ServoTest", group="DriverTest")
public class ServoTest extends _Autonomous {

    private _Servo _right;
    private States _state;
    private boolean _justEntered;


    @Override
    public void init() {
        Robot.setup(hardwareMap, telemetry);
        _right = new _Servo("bucketRight",Servo.Direction.REVERSE, 0, 1, 0,0.17, 90, 0.51, 180);
        _right.setDegree(0);
        _justEntered = true;
    }

    @Override
    public void loop(){
        _right.update();
        telemetry.addLine(String.valueOf(_right.getName()));

        switch(_state){
            case SET_POS:
                if(_justEntered){
                    _justEntered=false;
                }
                else if (!_right.isBusy()){
                    _right.setPosition(0.5);
                }
            case SET_DEG:
                if(_justEntered){
                    _justEntered = false;
                }
                else if(!_right.isBusy()){
                    _right.setDegree(45);
                }
            case SET_SLDEG:
                if(_justEntered){
                    _justEntered = false;
                }
                else if(!_right.isBusy()){
                    _right.setSlowDegree(45,0.5);
                }
            case SET_SLPOS:
                if(_justEntered){
                    _justEntered = false;
                }
                else if(!_right.isBusy()){
                    _right.setSlowPosition(0.7,0.8);
                }
                break;
        }
    }
    private enum States {
        SET_POS,
        SET_DEG,
        SET_SLPOS,
        SET_SLDEG
    }
}
