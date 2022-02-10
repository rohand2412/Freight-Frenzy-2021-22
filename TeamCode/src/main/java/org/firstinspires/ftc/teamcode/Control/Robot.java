package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivers._Drivetrain;
import org.firstinspires.ftc.teamcode.Drivers._IMU;
import org.firstinspires.ftc.teamcode.Drivers._Motor;
import org.firstinspires.ftc.teamcode.Drivers._OpenCV;
import org.firstinspires.ftc.teamcode.Drivers._PID;
import org.firstinspires.ftc.teamcode.Drivers._Servo;
import org.firstinspires.ftc.teamcode.Drivers._ServoGroup;
import org.firstinspires.ftc.teamcode.Drivers._TFOD;
import org.firstinspires.ftc.teamcode.Drivers._Vuforia;

public final class Robot {

    public static ElapsedTime runtime;
    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;

    private static _Drivetrain _drivetrain;
    private static _ServoGroup _bucket;
    private static _Motor _intake;
    private static _Motor _craneLift;
    private static _PID _craneLiftPID;
    private static _Motor _cranePivot;
    private static _PID _cranePivotPID;
    private static _Motor _carousel;
    private static _OpenCV _webcam;
    private static _Vuforia _vuforia;
    private static _TFOD _tfod;
    private static _IMU _imu;
    private static _IMU _craneIMU;

    public static final double MM_PER_INCH = 25.4;

    private static final double _TURN_OFFSET_POSITIVE = 18;
    private static final double _TURN_OFFSET_NEGATIVE = 15;
    private static final double _ENDGAME_CAROUSEL_SPEED = 0.5;

    private static boolean _isTurning = false;
    private static double _startAngle;
    private static double _turnDegrees;
    private static double _craneLiftSetPoint;
    private static double _cranePivotSetPoint;

    private Robot() {};

    public static void setup(HardwareMap centralHardwareMap, Telemetry centralTelemetry, SetupType... setupTypes) {
        runtime = new ElapsedTime();
        hardwareMap = centralHardwareMap;
        telemetry = centralTelemetry;

        StringBuilder setupSequence = new StringBuilder();
        for (SetupType type : setupTypes) {
            switch(type) {
                case Everything:
                    setupEverything();
                    break;
                case Drivetrain:
                    setupDrivetrain();
                    break;
                case Bucket:
                    setupBucket();
                    break;
                case Intake:
                    setupIntake();
                    break;
                case CraneLift:
                    setupCraneLift();
                    break;
                case CranePivot:
                    setupCranePivot();
                    break;
                case Carousel:
                    setupCarousel();
                    break;
                case OpenCV:
                    setupOpenCV();
                    break;
                case Vuforia:
                    setupVuforia();
                    break;
                case TFOD:
                    setupTFOD();
                    break;
                case IMU:
                    setupIMU();
                    break;
                case CraneIMU:
                    setupCraneIMU();
                    break;
            }

            setupSequence.append(type.name()).append(" ");
        }

        telemetry.addLine(setupSequence.toString());
    }

    private static void setupEverything() {
        setupVuforia();
        setupTFOD();
        setupIMU();
        setupCraneIMU();
        setupDrivetrain();
        setupBucket();
        setupIntake();
        setupCarousel();
        //OpenCV is just for testing, not actual runs
    }

    private static void setupDrivetrain() {
        double wheelDiameter = 96/MM_PER_INCH;
        _Motor fr = new _Motor("motorFR", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, wheelDiameter, true);
        _Motor fl = new _Motor("motorFL", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, wheelDiameter, true);
        _Motor br = new _Motor("motorBR", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, wheelDiameter, true);
        _Motor bl = new _Motor("motorBL", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, wheelDiameter, true);
        _drivetrain = new _Drivetrain(fr, fl, br, bl, 1.0);
    }

    private static void setupBucket() {
        _Servo left = new _Servo("bucketLeft", Servo.Direction.FORWARD, 0, 1, 0,
                0.17, 90, 0.51, 180);
        _Servo right = new _Servo("bucketRight", Servo.Direction.REVERSE, 0, 1, 0,
                0.17, 90, 0.51, 180);
        _bucket = new _ServoGroup(left, right);
    }

    private static void setupIntake() {
        _intake = new _Motor("intake", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, false);
    }

    private static void setupCraneLift() {
        _craneLift = new _Motor("craneLift", _Motor.Type.GOBILDA_30_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, false);
        _craneLiftPID = new _PID(() -> _craneIMU.getRoll(), (double data) -> _craneLift.runSpeed(data), () -> _craneLiftSetPoint,
                0.008, 0.01, 0, 0.006, 0.006, 0,
                _PID.ProportionalMode.MEASUREMENT, _PID.Direction.DIRECT, 50, -1, 1);
    }

    private static void setupCranePivot() {
        _cranePivot = new _Motor("cranePivot", _Motor.Type.GOBILDA_117_RPM, DcMotorSimple.Direction.REVERSE,
                DcMotor.ZeroPowerBehavior.BRAKE, false);
        _cranePivotPID = new _PID(() -> _craneIMU.getYaw(), (double data) -> _cranePivot.runSpeed(data), () -> _cranePivotSetPoint,
                0.0085, 0.015, 0, _PID.ProportionalMode.MEASUREMENT, _PID.Direction.DIRECT, 50, -1, 1);
    }

    private static void setupCarousel() {
        _carousel = new _Motor("carousel", _Motor.Type.GOBILDA_312_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, true);
    }

    private static void setupOpenCV() {
        _webcam = new _OpenCV("Webcam 1", 320, 240);
    }

    private static void setupVuforia() {
        _vuforia = new _Vuforia("Webcam 1");
    }

    private static void setupTFOD() {
        _tfod = new _TFOD(_vuforia.getVuforia(), 0.45f, true, 320, 1.1, 16.0/9.0,
                "FreightFrenzy_BCDM.tflite", new String[] {"Ball", "Cube", "Duck", "Marker"});
    }

    private static void setupIMU() {
        _imu = new _IMU("imu", false, true);
    }

    private static void setupCraneIMU() {
        _craneIMU = new _IMU("armImu", 50, true, false);
    }

    public static void update() {
        _drivetrain.update();
        _bucket.update();
        _intake.update();
        _craneLift.update();
        _craneLiftPID.update();
        _cranePivot.update();
        _cranePivotPID.update();
        _carousel.update();
        _imu.update();
        _craneIMU.update();

        if (_isTurning) {
            if (Math.abs(_turnDegrees) > Math.max(_TURN_OFFSET_POSITIVE, _TURN_OFFSET_NEGATIVE)) {
                if (_turnDegrees > 0 ? _imu.getYaw() - _startAngle >= _turnDegrees - _TURN_OFFSET_POSITIVE : _imu.getYaw() - _startAngle <= _turnDegrees + _TURN_OFFSET_NEGATIVE) {
                    _isTurning = false;
                }
            }
            else {
                if (_turnDegrees > 0 ? _imu.getYaw() - _startAngle >= _turnDegrees : _imu.getYaw() - _startAngle <= _turnDegrees) {
                    _isTurning = false;
                }
            }

            if (!_isTurning) {
                _drivetrain.stop();
            }
        }
    }

    public static void turn(double speed, double degrees, TurnAxis turnAxis) {
        if (!_isTurning && degrees != 0) {
            _isTurning = true;
            _startAngle = _imu.getYaw();
            degrees = _turnDegrees;

            switch (turnAxis) {
                case Center:
                    _drivetrain.runSpeed(speed, degrees > 0 ? _Drivetrain.Movements.cw : _Drivetrain.Movements.ccw);
                    break;
                case Back:
                    _drivetrain.runSpeed(speed, degrees > 0 ? _Drivetrain.Movements.cwback : _Drivetrain.Movements.ccwback);
                    break;
                case Front:
                    _drivetrain.runSpeed(speed, degrees > 0 ? _Drivetrain.Movements.cwfront : _Drivetrain.Movements.ccwfront);
                    break;
            }
        }
    }

    public static void setCraneLiftDegree(double degree) {
        _craneLiftSetPoint = degree;
    }

    public static void setCranePivotDegree(double degree) {
        _cranePivotSetPoint = degree;
    }

    public static _Drivetrain getDrivetrain() {
        return _drivetrain;
    }

    public static _ServoGroup getBucket() {
        return _bucket;
    }

    public static _Motor getIntake() {
        return _intake;
    }

    public static _PID getCraneLiftPID() {
        return _craneLiftPID;
    }

    public static _Motor getCraneLift() {
        return _craneLift;
    }

    public static _PID getCranePivotPID() {
        return _cranePivotPID;
    }

    public static _Motor getCranePivot() {
        return _cranePivot;
    }

    public static _Motor getCarousel() {
        return _carousel;
    }

    public static _OpenCV getWebcam() {
        return _webcam;
    }

    public static _Vuforia getVuforia() {
        return _vuforia;
    }

    public static _TFOD getTFOD() {
        return _tfod;
    }

    public static _IMU getIMU() {
        return _imu;
    }

    public static _IMU getCraneIMU() {
        return _craneIMU;
    }

    public static boolean isTurning() {
        return _isTurning;
    }

    public enum SetupType {
        Everything,
        Drivetrain,
        Bucket,
        Intake,
        CraneLift,
        CranePivot,
        Carousel,
        OpenCV,
        Vuforia,
        TFOD,
        IMU,
        CraneIMU
    }

    public enum TurnAxis {
        Front,
        Center,
        Back
    }
}