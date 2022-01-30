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
import org.firstinspires.ftc.teamcode.Drivers._Servo;
import org.firstinspires.ftc.teamcode.Drivers._ServoGroup;
import org.firstinspires.ftc.teamcode.Drivers._TFOD;
import org.firstinspires.ftc.teamcode.Drivers._Vuforia;

public final class Robot {

    public static ElapsedTime runtime;
    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;

    public static _Drivetrain drivetrain;
    public static _ServoGroup bucket;
    public static _Motor intake;
    public static _Motor craneLift;
    public static _Motor cranePivot;
    public static _Motor carousel;
    public static _OpenCV webcam;
    public static _Vuforia vuforia;
    public static _TFOD tfod;
    public static _IMU imu;

    public static final double MM_PER_INCH = 25.4;
    public static final double TURN_OFFSET_POSITIVE = 18;
    public static final double TURN_OFFSET_NEGATIVE = 15;

    public static boolean isTurning = false;
    public static double startAngle;
    public static double turnDegrees;

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
                case Crane:
                    setupCrane();
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
            }

            setupSequence.append(type.name()).append(" ");
        }

        telemetry.addLine(setupSequence.toString());
        telemetry.update();
    }

    private static void setupEverything() {
        setupVuforia();
        setupTFOD();
        setupIMU();
        setupDrivetrain();
        setupBucket();
        setupIntake();
        setupCarousel();
        //OpenCV is just for testing, not actual runs
    }

    private static void setupDrivetrain() {
        double wheelDiameter = 96/MM_PER_INCH;
        _Motor fr = new _Motor("motorFR", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, wheelDiameter);
        _Motor fl = new _Motor("motorFL", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, wheelDiameter);
        _Motor br = new _Motor("motorBR", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, wheelDiameter);
        _Motor bl = new _Motor("motorBL", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE, wheelDiameter);
        drivetrain = new _Drivetrain(fr, fl, br, bl, 1.0);
    }

    private static void setupBucket() {
        _Servo left = new _Servo("bucketLeft", Servo.Direction.FORWARD, 0, 1, 0,
                0.17, 90, 0.51, 180);
        _Servo right = new _Servo("bucketRight", Servo.Direction.REVERSE, 0, 1, 0,
                0.17, 90, 0.51, 180);
        bucket = new _ServoGroup(left, right);
    }

    private static void setupIntake() {
        intake = new _Motor("intake", _Motor.Type.GOBILDA_435_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void setupCrane() {
        craneLift = new _Motor("craneLift", _Motor.Type.GOBILDA_30_RPM, DcMotorSimple.Direction.REVERSE,
                DcMotor.ZeroPowerBehavior.BRAKE);
        cranePivot = new _Motor("cranePivot", _Motor.Type.GOBILDA_117_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void setupCarousel() {
        carousel = new _Motor("carousel", _Motor.Type.GOBILDA_312_RPM, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void setupOpenCV() {
        webcam = new _OpenCV("Webcam 1", 320, 240);
    }

    private static void setupVuforia() {
        vuforia = new _Vuforia("Webcam 1");
    }

    private static void setupTFOD() {
        tfod = new _TFOD(vuforia.getVuforia(), 0.45f, true, 320, 1.1, 16.0/9.0,
                "FreightFrenzy_BCDM.tflite", new String[] {"Ball", "Cube", "Duck", "Marker"}, true);
    }

    private static void setupIMU() {
        imu = new _IMU("imu", false);
    }

    public void update() {
        drivetrain.update();
        bucket.update();
        intake.update();
        craneLift.update();
        cranePivot.update();
        carousel.update();
        tfod.update();
        imu.update();

        if (isTurning) {
            if (Math.abs(turnDegrees) > Math.max(TURN_OFFSET_POSITIVE, TURN_OFFSET_NEGATIVE)) {
                if (turnDegrees > 0 ? imu.getYaw() - startAngle >= turnDegrees - TURN_OFFSET_POSITIVE : imu.getYaw() - startAngle <= turnDegrees + TURN_OFFSET_NEGATIVE) {
                    isTurning = false;
                }
            }
            else {
                if (turnDegrees > 0 ? imu.getYaw() - startAngle >= turnDegrees : imu.getYaw() - startAngle <= turnDegrees) {
                    isTurning = false;
                }
            }

            if (!isTurning) {
                drivetrain.stop();
            }
        }
    }

    public void turn(double speed, double degrees, TurnAxis turnAxis) {
        if (!isTurning && degrees != 0) {
            isTurning = true;
            startAngle = imu.getYaw();
            degrees = turnDegrees;

            switch (turnAxis) {
                case Center:
                    drivetrain.runSpeed(speed, degrees > 0 ? _Drivetrain.Movements.cw : _Drivetrain.Movements.ccw);
                    break;
                case Back:
                    drivetrain.runSpeed(speed, degrees > 0 ? _Drivetrain.Movements.cwback : _Drivetrain.Movements.ccwback);
                    break;
                case Front:
                    drivetrain.runSpeed(speed, degrees > 0 ? _Drivetrain.Movements.cwfront : _Drivetrain.Movements.ccwfront);
                    break;
            }
        }
    }

    public _Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public _ServoGroup getBucket() {
        return bucket;
    }

    public _Motor getIntake() {
        return intake;
    }

    public _Motor getCraneLift() {
        return craneLift;
    }

    public _Motor getCranePivot() {
        return cranePivot;
    }

    public _Motor getCarousel() {
        return carousel;
    }

    public _OpenCV getWebcam() {
        return webcam;
    }

    public _Vuforia getVuforia() {
        return vuforia;
    }

    public _TFOD getTFOD() {
        return tfod;
    }

    public _IMU getIMU() {
        return imu;
    }

    public boolean isTurning() {
        return isTurning;
    }

    public enum SetupType {
        Everything,
        Drivetrain,
        Bucket,
        Intake,
        Crane,
        Carousel,
        OpenCV,
        Vuforia,
        TFOD,
        IMU
    }

    public enum TurnAxis {
        Front,
        Center,
        Back
    }
}