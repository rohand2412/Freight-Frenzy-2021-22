package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_COREHEXMOTOR_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_GOBUILDA435RPM_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.claws;
import static org.firstinspires.ftc.teamcode.Control.Constants.collections;
import static org.firstinspires.ftc.teamcode.Control.Constants.feederLeftS;
import static org.firstinspires.ftc.teamcode.Control.Constants.feederRightS;
import static org.firstinspires.ftc.teamcode.Control.Constants.flys;
import static org.firstinspires.ftc.teamcode.Control.Constants.imuS;
import static org.firstinspires.ftc.teamcode.Control.Constants.lifters;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.pincher;
import static org.firstinspires.ftc.teamcode.Control.Constants.Backs;
import static org.firstinspires.ftc.teamcode.Control.Constants.Rights;
import static org.firstinspires.ftc.teamcode.Control.Constants.Fronts;
import static org.firstinspires.ftc.teamcode.Control.Constants.Lefts;
//import static org.firstinspires.ftc.teamcode.Control.Constants.rightBacks;
//import static org.firstinspires.ftc.teamcode.Control.Constants.rightFronts;
import static org.firstinspires.ftc.teamcode.Control.Constants.shooterLeftS;
import static org.firstinspires.ftc.teamcode.Control.Constants.shooterRightS;
import static org.firstinspires.ftc.teamcode.Control.Constants.whacker;
//import static org.firstinspires.ftc.teamcode.Control.Constants.backSenseS;
//import static org.firstinspires.ftc.teamcode.Control.Constants.leftSenseS;
//import static org.firstinspires.ftc.teamcode.Control.Constants.frontSenseS;
//import static org.firstinspires.ftc.teamcode.Control.Constants.rightSenseS;


public class Goal {

    public Goal(HardwareMap hardwareMap, ElapsedTime runtime, Central central, setupType... setup) throws InterruptedException {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.central = central;

        StringBuilder i = new StringBuilder();

        for (setupType type: setup) {
            switch (type) {
                case autonomous:
                    setupDrivetrain();
                    setupStorage();
                    setupCollection();
                    setupFly();
                    setupWobbleGoalSystem();
                    setupUltra();
                    setupIMU();
                    break;
                case teleop:
                    setupDrivetrain();
                    setupStorage();
                    setupCollection();
                    setupFly();
                    setupWobbleGoalSystem();
                    setupUltra();
                    setupIMU();
                    break;
                case storage:
                    setupStorage();
                    setupFly();
                    break;
                case wobblegoal:
                    setupWobbleGoalSystem();
                    break;
                case flywheel:
                    setupFly();
                    break;
                case collectionsystem:
                    setupCollection();
                    break;
                case drivetrain_system:
                    setupDrivetrain();
                    break;
                case ultra:
                    setupUltra();
                    break;
                case imu:
                    setupIMU();
                    break;
                case openCV:
                    setupOpenCV();
                    break;

                case shooter:
                    setupShooter();
                    break;

            }

            i.append(type.name()).append(" ");

        }
        central.telemetry.addLine(i.toString());
        central.telemetry.update();

    }

    // important non-confdiguration field
    public Orientation angles;
    public ElapsedTime runtime;     //set in constructor to the runtime of running class
    public Central central;
    public HardwareMap hardwareMap;

    public boolean target = false;
    public boolean moving1 = false;
    public boolean moving2 = false;
    public boolean moving3 = false;
    public boolean stop = false;
    public boolean straight = false;
    public int x = 0;
    public int y = 0;
    public int blockNumber = 0;

    public int[] wheelAdjust = {-1, -1, -1, -1};

    public static double speedAdjust = 20.0 / 41.0;
    public static double yToXRatio = 1.25;

    public void setWheelAdjust(int fr, int fl, int br, int bl) {
        wheelAdjust[0] = fr;
        wheelAdjust[1] = fl;
        wheelAdjust[2] = br;
        wheelAdjust[3] = bl;
    }
    //----specfic non-configuration fields
    //none rnh


    // Vuforia Variables
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false  ;


    public final String VUFORIA_KEY =
            " AYzLd0v/////AAABmR035tu9m07+uuZ6k86JLR0c/MC84MmTzTQa5z2QOC45RUpRTBISgipZ2Aop4XzRFFIvrLEpsop5eEBl5yu5tJxK6jHbMppJyWH8lQbvjz4PAK+swG4ALuz2M2MdFXWl7Xh67s/XfIFSq1UJpX0DgwmZnoDCYHmx/MnFbyxvpWIMLZziaJqledMpZtmH11l1/AS0oH+mrzWQLB57w1Ur0FRdhpxcrZS9KG09u6I6vCUc8EqkHqG7T2Zm4QdnytYWpVBBu17iRNhmsd3Ok3w8Pn22blBYRo6dZZ8oscyQS1ZtilM1YT49ORQHc8mu/BMWh06LxdstWctSiGiBV0+Wn3Zk++xQ750c64lg3QLjNkXc";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;

    // Class Members
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    public WebcamName webcamName = null;


    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    public VuforiaTrackables targetsUltimateGoal;
    public boolean targetVisible = false;
    public OpenGLMatrix robotFromCamera;
    public int cameraMonitorViewId;
    public VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


    //OpenCV Variables
    public OpenCvWebcam webcam;


    //----------------CONFIGURATION FIELDS--------------------
    public DcMotor[] drivetrain;   //set in motorDriveMode() for drivetrain movement functions

    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;

    public DcMotor fly;
    public DcMotor collection;
    public DcMotor claw;
    public Servo whack;
    public Servo pinch;
    public Servo lifter;
    public ModernRoboticsI2cRangeSensor Back, Right, Front, Left;

    public DcMotor shooterLeft;
    public DcMotor shooterRight;

    public CRServo feederLeft;
    public CRServo feederRight;

    public BNO055IMUImpl imu;


//    public ModernRoboticsI2cRangeSensor leftSense;
//    public ModernRoboticsI2cRangeSensor frontSense;
//    public Rev2mDistanceSensor rightfrontSense;
//    public Rev2mDistanceSensor rightbackSense;
//    public ModernRoboticsI2cRangeSensor backSense;


    public double StrafetoTotalPower = 2.0/3.0;

    //----       IMU        ----

    public BNO055IMUImpl.Parameters imuparameters = new BNO055IMUImpl.Parameters();
    public Orientation current;
    public static boolean isnotstopped;
    public float initorient;

    public void setupIMU() throws InterruptedException {
        imuparameters.angleUnit = BNO055IMUImpl.AngleUnit.DEGREES;
        imuparameters.accelUnit = BNO055IMUImpl.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        imuparameters.loggingEnabled = true; //copypasted from BNO055IMU sample code, no clue what this does
        imuparameters.loggingTag = "imu"; //copypasted from BNO055IMU sample code, no clue what this does
        imu = hardwareMap.get(BNO055IMUImpl.class, imuS);
        imu.initialize(imuparameters);
        initorient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        central.telemetry.addData("IMU status", imu.getSystemStatus());
        central.telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


    }




    public void setupDrivetrain() throws InterruptedException {
        motorFR = motor(motorFRS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL = motor(motorFLS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR = motor(motorBRS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL = motor(motorBLS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveMode(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }


    public void setupStorage() throws InterruptedException {
        whack = servo(whacker, Servo.Direction.FORWARD, 0, 1, 0);
        lifter = servo(lifters, Servo.Direction.FORWARD, 0, 1 , 1);
        // teleop .98
        encoder(EncoderMode.OFF, fly);
    }

    public void setupCollection() throws InterruptedException {
        collection = motor(collections, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        encoder(EncoderMode.ON, collection);

    }

    public void setupFly() throws InterruptedException {
        fly = motor(flys, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);

        encoder(EncoderMode.OFF, fly);


    }

    public void setupShooter() throws InterruptedException{
        shooterLeft = motor(shooterLeftS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight = motor(shooterRightS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        feederLeft = servo(feederLeftS, CRServo.Direction.FORWARD, 0);
        feederRight = servo(feederRightS, DcMotorSimple.Direction.FORWARD, 0);
    }

    public void setupWobbleGoalSystem() throws InterruptedException {
        claw = motor(claws, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        pinch = servo(pincher, Servo.Direction.FORWARD, 0, 1, 0.2);

        encoder(EncoderMode.OFF, claw);

    }

    public void setupUltra() throws InterruptedException {
        Back = ultrasonicSensor(Backs);
        Right = ultrasonicSensor(Rights);
        Front = ultrasonicSensor(Fronts);
        Left = ultrasonicSensor(Lefts);
    }

    public void setupOpenCV() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
//    public void setupMapping() throws InterruptedException {
//
//        leftSense = ultrasonicSensor(leftSenseS);
//        //frontSense = ultrasonicSensor(frontSenseS);
//        rightfrontSense = therealUS(rightfrontSenseS);
//        rightbackSense = therealUS(rightbackSenseS);
//        backSense = ultrasonicSensor(backSenseS);
//    }

    public void setupVuforia() throws InterruptedException {
        float phoneXRotate    = 0;
        float phoneYRotate    = 0;
        float phoneZRotate    = 0;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
    }




    //-----------------------HARDWARE SETUP FUNCTIONS---------------------------------------
    public DcMotor motor(String name, DcMotor.Direction directionm, DcMotor.ZeroPowerBehavior zeroPowerBehavior) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setPower(0);
        return motor;
    }

    public Servo servo(String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException {
        Servo servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public CRServo servo(String name, DcMotorSimple.Direction direction, double startSpeed) throws InterruptedException {
        CRServo servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);

        servo.setPower(startSpeed);
        return servo;
    }
    public ColorSensor colorSensor(String name, boolean ledOn) throws InterruptedException {
        ColorSensor sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        central.telemetry.addData("Beacon Red Value: ", sensor.red());
        central.telemetry.update();

        return sensor;
    }
    public ModernRoboticsI2cRangeSensor ultrasonicSensor(String name) throws InterruptedException {

        return hardwareMap.get(ModernRoboticsI2cRangeSensor.class, name);
    }
    public Rev2mDistanceSensor therealUS(String name) throws InterruptedException {
        return hardwareMap.get(Rev2mDistanceSensor.class, name);
    }

    public ModernRoboticsI2cColorSensor MRColor(String name) throws InterruptedException {
        return hardwareMap.get(ModernRoboticsI2cColorSensor.class, name);

    }

    public ModernRoboticsAnalogOpticalDistanceSensor realUS(String name) throws InterruptedException {
        return hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, name);
    }

    public void encoder(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                central.idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

    }

    public void motorDriveMode(EncoderMode mode, DcMotor... motor) throws InterruptedException {

        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                central.idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

        this.drivetrain = motor;

    }

    public void driveTrainEncoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement) throws InterruptedException {

        int[] targets = new int[drivetrain.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_GOBUILDA435RPM_INCH);
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:drivetrain){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();

                for (int i = 0; i < drivetrain.length; i++) {
                    DcMotor motor = drivetrain[i];
                    if (!motor.isBusy() && signs[i] != 0) {
                        x = false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: drivetrain){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }

    public void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement, DcMotor... motors) throws InterruptedException {

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_GOBUILDA435RPM_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }

    public void PIDFly(double totalTime, double targetPow, double targetSpeed, double p, double i, double d, double sleepTime, double correct, double bias){
        double initTime = runtime.seconds();
        double lastError = 0;
        double lastIntegral = 0;
        double lastTime = initTime;
        double lastCount = fly.getCurrentPosition();

        double st = sleepTime*1000;


        while(runtime.seconds()-initTime<totalTime){
            double timeElapsed = runtime.seconds() - lastTime;
            double countChange = fly.getCurrentPosition() - lastCount;


            double error = targetSpeed - Math.abs(countChange/timeElapsed);
            double integral = lastIntegral + error * (timeElapsed);
            double derivative = (error - lastError)/(timeElapsed);

            double total_correct = p * error + i * integral + d * derivative + bias;
/*
            central.telemetry.addData("speed value", targetPow + (-1*(total_correct*correct)));
            central.telemetry.addData("p correct", p*error);
            central.telemetry.addData("i correct", i*integral);
            central.telemetry.addData("d correct", d*derivative);
            central.telemetry.addData("velocity error", error);
            central.telemetry.addData("integral", integral);
            central.telemetry.addData("derivative", derivative);
            central.telemetry.addData("count change", countChange);
            central.telemetry.addData("time elapsed", timeElapsed);
            central.telemetry.addData("Actual velocity", Math.abs(countChange/timeElapsed));

            central.telemetry.update();
*/
            lastError = error;
            lastIntegral = integral;
            lastTime += timeElapsed;
            lastCount += countChange;

            if (targetPow + (-1*(total_correct*correct)) > -1) {

                fly.setPower(targetPow + (-1*(total_correct * correct)));
                central.sleep((long)st);
            }
            else {
                fly.setPower(-1);
                central.sleep((long)st);
            }

 /*           whack.setPosition(0.45);
            central.sleep(500);


            whack.setPosition(0);
            central.sleep(500);
*/

//            p *= 2;
//            i *= 2;
//            d *= 2;
        }
    }

    public double velocityFly(){
        int curr = fly.getCurrentPosition();
        double initTime = runtime.seconds();
        while(runtime.seconds()-initTime<0.01){
        }
        int newPos = fly.getCurrentPosition();
        return ((double)(newPos-curr)/0.01);
    }

    public void encodeCoreHexMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement, DcMotor... motors) throws InterruptedException {

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_COREHEXMOTOR_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }

    //------------------DRIVETRAIN TELEOP FUNCTIONS------------------------------------------------------------------------
    public void driveTrainMovement(double speed, movements movement) throws InterruptedException {
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
    }
    public void driveTrainMovement(double... speed) throws InterruptedException {

        for (int i = 0; i < drivetrain.length; i++) {
            drivetrain[i].setPower(speed[i]);
        }
    }
    public void driveTrainTimeMovement(double speed, movements movement, long duration, long waitAfter) throws InterruptedException {
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
        central.sleep(duration);
        stopDrivetrain();
        central.sleep(waitAfter);
    }

    public void anyMovement(double speed, movements movement, DcMotor... motors) throws InterruptedException {
        double[] signs = movement.getDirections();
        for (DcMotor motor: motors){
            int x = Arrays.asList(motors).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
    }
    public void anyMovementTime(double speed, movements movement, long duration, DcMotor... motors) throws InterruptedException {
        double[] signs = movement.getDirections();
        for (DcMotor motor: motors){
            int x = Arrays.asList(motors).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
        central.sleep(duration);
        for (DcMotor motor: motors){
            motor.setPower(0);

        }
    }
    public void stopDrivetrain() throws InterruptedException {
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }

    public void powerMotors(double speed, long time, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
        central.sleep(time);
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    // IMU Movements
    public void turn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = getDirection();

        double end = (start + ((direction == turnside.cw) ? target : -target) + 360) % 360;

        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (InterruptedException e) {
            isnotstopped = false;
        }

        while (((calculateDifferenceBetweenAngles(getDirection(), end) > 1 && turnside.cw == direction) || (calculateDifferenceBetweenAngles(getDirection(), end) < -1 && turnside.ccw == direction)) && central.opModeIsActive() ) {
            central.telemetry.addLine("First Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Difference: ", (calculateDifferenceBetweenAngles(end, getDirection())));
            central.telemetry.update();
        }
        try {
            stopDrivetrain();
        } catch (InterruptedException e) {
        }
        central.sleep(5000);

        while (calculateDifferenceBetweenAngles(getDirection(), end) < -0.25 && central.opModeIsActive()) {
            driveTrainMovement(0.1, (direction == turnside.cw) ? movements.ccw : movements.cw);
            central.telemetry.addLine("Correctional Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Diffnce: ", calculateDifferenceBetweenAngles(end, getDirection()));
            central.telemetry.update();

        }
        stopDrivetrain();
        central.sleep(5000);

        central.telemetry.addLine("Completed");
        central.telemetry.addData("IMU Inital: ", start);
        central.telemetry.addData("IMU Final Projection: ", end);
        central.telemetry.addData("IMU Orient: ", getDirection());
        central.telemetry.addData("IMU Diffnce: ", calculateDifferenceBetweenAngles(end, getDirection()));
        central.telemetry.update();
        central.sleep(5000);
    }
    public void teleturn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = getDirection();

        double end = (start + ((direction == turnside.cw) ? target : -target) + 360) % 360;

       /* isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (InterruptedException e) {
            isnotstopped = false;
        }

        while ((((end - getDirection()) > 1 && turnside.cw == direction) || (turnside.cw != direction && end - getDirection() < -1)) && central.opModeIsActive() && isnotstopped) {
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.update();
        }
        try {
            stopDrivetrain();
        } catch (InterruptedException e) {
        }


        */

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle > 0) {
            while (angles.firstAngle > -1) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveTrainMovement(.5, Goal.movements.ccw);
            }
        }
        if (angles.firstAngle < 0) {
            while (angles.firstAngle < 1) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                driveTrainMovement(.5, Goal.movements.cw);
            }
        }

        double r = end-getDirection();
        boolean x;
        if(r>1){
            x=true;
        }
        else{
            x=false;
        }
        boolean y = false;

        while (Math.abs(end - getDirection()) > 1 && central.opModeIsActive()){
            r = end-getDirection();
            if(r>1){
                y=true;
            }
            else{
                y=false;
            }
            if(x!=y){
                break;
            }
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Target: ", target);
            central.telemetry.update();
            driveTrainMovement(.05, (direction == turnside.cw) ? movements.ccw : movements.cw);

        }
        stopDrivetrain();

    }


    public void absturn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException {

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = 0;

        double end = (start + ((direction == turnside.cw) ? target : -target) + 360) % 360;

        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (InterruptedException e) {
            isnotstopped = false;
        }

        while (((calculateDifferenceBetweenAngles(getDirection(), end) > 1 && turnside.cw == direction) || (calculateDifferenceBetweenAngles(getDirection(), end) < -1 && turnside.ccw == direction)) && central.opModeIsActive() ) {
            central.telemetry.addLine("First Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Difference: ", end - getDirection());
            central.telemetry.update();
        }
        try {
            stopDrivetrain();
        } catch (InterruptedException e) {
        }

        while (calculateDifferenceBetweenAngles(end, getDirection()) > 1 && central.opModeIsActive()){
            driveTrainMovement(0.05, (direction == turnside.cw) ? movements.ccw : movements.cw);
            central.telemetry.addLine("Correctional Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Diffnce: ", end - getDirection());
            central.telemetry.update();
        }
        stopDrivetrain();
        central.telemetry.addLine("Completed");
        central.telemetry.addData("IMU Inital: ", start);
        central.telemetry.addData("IMU Final Projection: ", end);
        central.telemetry.addData("IMU Orient: ", getDirection());
        central.telemetry.addData("IMU Diffnce: ", end - getDirection());
        central.telemetry.update();
    }

    public double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle) // negative is secondAngle ccw relative to firstAngle
    {
        double difference = secondAngle - firstAngle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return -difference;
    }

    public double getDirection(){
        return (this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle-initorient+720)%360;
    }



    public enum EncoderMode{
        ON, OFF;
    }
    public enum setupType{
        autonomous, teleop, collectionsystem, storage, flywheel, drivetrain_system, wobblegoal, ultra, imu, openCV, shooter;
    }



    //-------------------SET FUNCTIONS--------------------------------
    public void setCentral(Central central) {
        this.central = central;
    }
    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }

    //-------------------CHOICE ENUMS-------------------------
    public enum movements
    {
        // FR FL BR BL
        left(1, 1, -1, -1),
        right(-1, -1, 1, 1),
        forward(1, -1, 1, -1),
        backward(-1, 1, -1, 1),
        br(0, -1, 1, 0),
        bl(1, 0, 0, -1),
        tl(0, 1, -1, 0),
        tr(-1, 0, 0, 1),
        ccw(-1, -1, -1, -1),
        cw(1, 1, 1, 1),
        cwback(-1, -1, 0, 0),
        ccwback(1, 1, 0, 0),
        cwfront(0, 0, -1, -1),
        ccwfront(0, 0, 1, 1),
        linearUp(1),
        linearDown(-1),
        clawOut(-1),
        clawIn(1),
        shootForward(1, -1),
        shootBackward(-1, 1),
        feederForward(1, -1),
        feederBackward(-1, 1);



        private final double[] directions;

        movements(double... signs) {
            this.directions = signs;
        }

        public double[] getDirections() {
            return directions;
        }
    }


    public enum turnside {
        ccw, cw
    }

    /**
     * Finds the set of two direction speeds at the mecanum's movement angles that create a vector movement towards the desired angle
     * @param speed speed of movement
     * @param angleDegrees angle (in degrees) of movement
     * @return array of motor directions
     */
    public static double[] anyDirection(double speed, double angleDegrees) {
        double theta = Math.toRadians(angleDegrees);
        double beta = Math.atan(yToXRatio);

        double v1 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) + speed * Math.cos(theta) / Math.cos(beta));
        double v2 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) - speed * Math.cos(theta) / Math.cos(beta));

        double[] retval = {v1, v2};
        return retval;
    }

    public static double[] anyDirectionRadians(double speed, double angleRadians) {
        double theta = angleRadians;
        double beta = Math.atan(yToXRatio);

        double v1 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) + speed * Math.cos(theta) / Math.cos(beta));
        double v2 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) - speed * Math.cos(theta) / Math.cos(beta));

        double[] retval = {v1, v2};
        return retval;
    }

    public void driveTrainMovementAngle(double speed, double angle) {

        double[] speeds = anyDirection(speed, angle);
        motorFR.setPower(movements.forward.directions[0] * speeds[0]);
        motorFL.setPower(movements.forward.directions[1] * speeds[1]);
        motorBR.setPower(movements.forward.directions[2] * speeds[1]);
        motorBL.setPower(movements.forward.directions[3] * speeds[0]);

    }

    public void driveTrainMovementAngleRadians(double speed, double angle) {

        double[] speeds = anyDirectionRadians(speed, angle);
        motorFR.setPower(movements.forward.directions[0] * speeds[0]);
        motorFL.setPower(movements.forward.directions[1] * speeds[1]);
        motorBR.setPower(movements.forward.directions[2] * speeds[1]);
        motorBL.setPower(movements.forward.directions[3] * speeds[0]);

    }

    public enum axis {
        front, center, back
    }


    public void driveTrainIMUSwingTurnMovement(double speed, movements movement, long waitAfter, double rotationDegrees, double rotationfactor, turnside rotDir) throws InterruptedException{
        double[] signs = movement.getDirections();
        rotationDegrees = 360- rotationDegrees;
        double start = getDirection();

        double end = (start + ((rotDir == turnside.cw) ? rotationDegrees : -rotationDegrees) + 360) % 360;
        double[] speedValues = anyDirection(speed, -90 - start + getDirection());
        double[] speeds= new double[4];
        for (int i = 0; i < drivetrain.length; i++) {
            if (i == 0 || i == 4) {
                speeds[i] = (speedValues[1]);
            } else {
                speeds[i] = (speedValues[0]);
            }
        }


        while ((((end - getDirection()) > 1 && turnside.cw == rotDir) || (turnside.cw != rotDir && end - getDirection() < -1)) && central.opModeIsActive()) {
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());

            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setPower(signs[x] * speeds[x] + rotationfactor * movements.valueOf(rotDir.name()).getDirections()[x]);
                central.telemetry.addData("motor " + x, signs[x] * speeds[x] + rotationfactor * movements.valueOf(rotDir.name()).getDirections()[x]);

            }
            central.telemetry.update();
        }

        stopDrivetrain();
        while (Math.abs(end - getDirection()) > 1 && central.opModeIsActive()){
            driveTrainMovement(0.3, (rotDir == turnside.cw) ? movements.ccw : movements.cw);
        }


        stopDrivetrain();
        central.sleep(waitAfter);
    }
    public void driveTrainIMUSwingTurnMovementOrig(double speed, movements movement, long waitAfter, int rotationDegrees, double rotationfactor, turnside rotDir) throws InterruptedException{
        double[] signs = movement.getDirections();

        double start = getDirection();

        double end = (start + ((rotDir == turnside.cw) ? rotationDegrees : -rotationDegrees) + 360) % 360;



        while ((((end - getDirection()) > 1 && turnside.cw == rotDir) || (turnside.cw != rotDir && end - getDirection() < -1)) && central.opModeIsActive()) {
            double[] speedValues = anyDirection(speed, 90 + start - getDirection());
            double[] speeds= new double[4];
            for (int i = 0; i < drivetrain.length; i++) {
                if (i == 0 || i == 3) {
                    speeds[i] = (speedValues[0]);
                } else {
                    speeds[i] = (speedValues[1]);
                }
            }
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());

            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setPower(speeds[x] + rotationfactor * -movements.valueOf(rotDir.name()).getDirections()[x]);
                central.telemetry.addData("motor " + x, speeds[x] + rotationfactor * -movements.valueOf(rotDir.name()).getDirections()[x]);

            }
            central.telemetry.update();
        }

        stopDrivetrain();
        while (Math.abs(end - getDirection()) > 1 && central.opModeIsActive()){
            driveTrainMovement(0.1, (rotDir == turnside.cw) ? movements.ccw : movements.cw);
        }


        stopDrivetrain();
        central.sleep(waitAfter);
    }

    public void driveTrainIMUSuperStrafeMovement(double speed, movements movement, long waitAfter, int rotationDegrees, double rotationfactor, turnside rotDir) throws InterruptedException{
        double[] signs = movement.getDirections();

        double start = getDirection();

        double end = (start + ((rotDir == turnside.cw) ? rotationDegrees : -rotationDegrees) + 360) % 360;
        double[] speedValues = anyDirection(speed, 90 + start - getDirection());
        double[] speeds= new double[4];
        for (int i = 0; i < drivetrain.length; i++) {
            if (i == 0 || i == 3) {
                speeds[i] = (speedValues[0]);
            } else {
                speeds[i] = (speedValues[1]);
            }
        }

        int p = 0;

        boolean rotate = false;
        while ((((end - getDirection()) > 1 && turnside.cw == rotDir) || (turnside.cw != rotDir && end - getDirection() < -1)) && central.opModeIsActive()) {
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            if (p % 10 == 0){
                rotate = !rotate;
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                if (rotate) {
                    motor.setPower(rotationfactor * movements.valueOf(rotDir.name()).getDirections()[x]);
                    central.telemetry.addData("motor " + x, rotationfactor * movements.valueOf(rotDir.name()).getDirections()[x]);
                }
                else {
                    motor.setPower(speeds[x]);
                    central.telemetry.addData("motor " + x, speeds[x]);
                }

            }
            central.telemetry.update();
            p++;
        }

        stopDrivetrain();
        while (Math.abs(end - getDirection()) > 1 && central.opModeIsActive()){
            driveTrainMovement(0.1, (rotDir == turnside.cw) ? movements.ccw : movements.cw);
        }


        stopDrivetrain();
        central.sleep(waitAfter);
    }

}
