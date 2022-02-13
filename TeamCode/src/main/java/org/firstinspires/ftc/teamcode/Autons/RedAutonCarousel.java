package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Control.Robot;
import org.firstinspires.ftc.teamcode.Control._Autonomous;
import org.firstinspires.ftc.teamcode.Drivers._TFOD;

import java.util.List;

@Autonomous(group="Auton")
public class RedAutonCarousel extends _Autonomous {

    private _TFOD.ValidRecognition _validRecognition;
    private List<Recognition> _recognitions;
    private TeamElementLocations _location;
    private boolean _tfodRight = true;

    @Override
    public void init() {
        Robot.setup(hardwareMap, telemetry, Robot.SetupType.Autonomous);
        Robot.setFieldSide(Robot.FieldSide.BLUE); //Auton flip
        Robot.getBucket().setDegree(45);
        Robot.setCraneLiftDegree(Robot.CRANE_COLLECTION_HOLD.CRANE_LIFT_DEGREE);
        Robot.setCranePivotDegree(Robot.CRANE_COLLECTION_HOLD.CRANE_PIVOT_DEGREE);

        _validRecognition = recognition ->
                ((recognition.getWidth() * recognition.getHeight()) < (320.0 * 640.0 / 8.0))
                && (recognition.getWidth()/recognition.getHeight()) > 1.0;
    }

    @Override
    public void init_loop() {
        Robot.update();

        _recognitions = Robot.getTFOD().getLatestRecognitions();
        if (_recognitions == null) {
            _location = _tfodRight ? TeamElementLocations.LEFT : TeamElementLocations.RIGHT;
        }
        else if (_recognitions.size() == 0) {
            _location = _tfodRight ? TeamElementLocations.LEFT : TeamElementLocations.RIGHT;
        }
        else {
            if (Robot.getTFOD().countValidLabel(_validRecognition, "Marker") == 1) {
                if ((Robot.getTFOD().getRecognitionValidLabel(_validRecognition, "Marker").getLeft()
                    + Robot.getTFOD().getRecognitionValidLabel(_validRecognition, "Marker").getRight())/2.0 > 320) {
                    _location = _tfodRight ? TeamElementLocations.MIDDLE : TeamElementLocations.LEFT;
                }
                else {
                    _location = _tfodRight ? TeamElementLocations.RIGHT : TeamElementLocations.MIDDLE;
                }
            }
            else {
                _location = _tfodRight ? TeamElementLocations.LEFT : TeamElementLocations.RIGHT;
            }
        }

        telemetry.addLine(_location != null ? _location.name() : "null");
        telemetry.addLine(_recognitions != null ? _recognitions.toString() : "null");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    private enum TeamElementLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}