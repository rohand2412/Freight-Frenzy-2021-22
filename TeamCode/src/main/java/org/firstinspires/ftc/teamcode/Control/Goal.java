package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_DEGREE_GOBILDA_30_RPM;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_DEGREE_REV_CORE_HEX_MOTOR;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_INCH_LINEAR_SLIDE_MOTOR;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_INCH_REV_CORE_HEX_MOTOR;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_INCH_GOBILDA_435_RPM;
import static org.firstinspires.ftc.teamcode.Control.Constants.FFBCDM_LABELS;
import static org.firstinspires.ftc.teamcode.Control.Constants.FFBCDM_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.Control.Constants.bucketLeftS;
import static org.firstinspires.ftc.teamcode.Control.Constants.bucketRightS;
import static org.firstinspires.ftc.teamcode.Control.Constants.carouselRightS;
import static org.firstinspires.ftc.teamcode.Control.Constants.imuS;
import static org.firstinspires.ftc.teamcode.Control.Constants.intakeClawS;
import static org.firstinspires.ftc.teamcode.Control.Constants.intakeLinearSlideS;
import static org.firstinspires.ftc.teamcode.Control.Constants.intakePivotS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.intakeS;
import static org.firstinspires.ftc.teamcode.Control.Constants.backUltraS;
import static org.firstinspires.ftc.teamcode.Control.Constants.rightUltraS;
import static org.firstinspires.ftc.teamcode.Control.Constants.leftUltraS;
import static org.firstinspires.ftc.teamcode.Control.Constants.frontUltraS;
import static org.firstinspires.ftc.teamcode.Control.Constants.carouselLeftS;

import static org.firstinspires.ftc.teamcode.Control.Constants.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmPerInch;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmFTCFieldWidth;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmFTCHalfFieldWidth;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmFTCQuadFieldWidth;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmTargetHeight;
import static org.firstinspires.ftc.teamcode.Control.Constants.CAMERA_CHOICE;
import static org.firstinspires.ftc.teamcode.Control.Constants.PHONE_IS_PORTRAIT;
import static org.firstinspires.ftc.teamcode.Control.Constants.webcamS;

public class Goal {

    /** Initialized in constructor **/
    public ElapsedTime runtime;
    public Central central;
    public HardwareMap hardwareMap;

    public static double yToXRatio = 1.0;

    public ModernRoboticsI2cRangeSensor backUltrasonic;
    public ModernRoboticsI2cRangeSensor rightUltrasonic;
    public ModernRoboticsI2cRangeSensor leftUltrasonic;
    public ModernRoboticsI2cRangeSensor frontUltrasonic;

    public OpenCvWebcam webcam;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    /** ---------------------------- DRIVETRAIN ----------------------------- **/
    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;
    public DcMotor intake;
    public DcMotor intakeLinearSlide;
    public Servo intakePivot;
    public Servo intakeClaw;
    public Servo bucketLeft;
    public Servo bucketRight;
    public DcMotor carouselRight;
    public DcMotor carouselLeft;

    public final double carouselTele = 0.9;
    public final double carouselAuton = 0.3;

    /** Set in motorDriveMode() for drivetrain movement functions **/
    public DcMotor[] drivetrain;

    /** -------------------------------- IMU ------------------------------- **/
    public BNO055IMU imu;

    /** IMU params **/
    public Orientation angles;
    public double yaw = 0;
    public double yawRaw = 0;
    public double lastYawRaw = 0;
    public final double overflowThreshold = 300;
    public final double fullCircleDeg = 360;

    public final double turnOffsetPositive = 25;
    public final double turnOffsetNegative = 16;

    /** Temp variables **/
    public static boolean isnotstopped;
    public float initorient;

    public Goal(HardwareMap hardwareMap, ElapsedTime runtime, Central central, setupType... setup) throws InterruptedException {
        //Update instance variables
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.central = central;

        //For sending to control hub via telemetry
        StringBuilder i = new StringBuilder();

        //Take action based on setup mode(s)
        for (setupType type: setup) {
            switch (type) {
                case autonomous:
                    setupAuton();
                    break;
                case teleop:
                    setupTeleop();
                    break;
                case drivetrain_system:
                    setupDrivetrain();
                    break;
                case intake:
                    setupIntake();
                    break;
                case crane:
                    setupCrane();
                    break;
                case carousel:
                    setupCarousel();
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
                case webcamStream:
                    setupWebcamStream();
                    break;
                case vuforia:
                    setupVuforia();
                    break;
                case tfod:
                    setupTFOD();
                    break;
            }

            //Update string with setup type
            i.append(type.name()).append(" ");

        }

        //Send string to control hub
        central.telemetry.addLine(i.toString());
        central.telemetry.update();

    }

    //function setups based on autonomous
    public void setupAuton() throws InterruptedException {
        setupIMU();
        setupDrivetrain();
        setupIntake();
        setupCarousel();
        setupVuforia();
        setupTFOD();
    }

    //function setups based on manual control
    public void setupTeleop() throws InterruptedException {
        setupIMU();
        setupDrivetrain();
        setupIntake();
        setupCarousel();
        setupVuforia();
        setupTFOD();
    }

    public void setupIMU() throws InterruptedException {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, imuS);
        imu.initialize(parameters);
    }

    //sets each drivetrain motor to correct direction and makes it brake when no power given
    public void setupDrivetrain() throws InterruptedException {
        motorFR = motor(motorFRS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL = motor(motorFLS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR = motor(motorBRS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL = motor(motorBLS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        setDriveTrain(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }

    //sets up the intake motors (intakeS), sets desired direction and brakes w/ no power
    public void setupIntake() throws InterruptedException {
        intake = motor(intakeS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLinearSlide = motor(intakeLinearSlideS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        intakePivot = servo(intakePivotS, Servo.Direction.REVERSE, 0, 1, 0);
        intakeClaw = servo(intakeClawS, Servo.Direction.FORWARD, 0.073, 0.1525, 1);
    }

    public void setupCrane() throws InterruptedException {
        bucketLeft = servo(bucketLeftS, Servo.Direction.FORWARD, 0, 1, 0);
        bucketRight = servo(bucketRightS, Servo.Direction.REVERSE, 0, 1, 0);
    }

    //sets motor responsible for spinning carousel
    public void setupCarousel() throws InterruptedException {
        carouselRight = motor(carouselRightS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        carouselLeft = motor(carouselLeftS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //ultrasonic sensors defined&setup
    public void setupUltra() throws InterruptedException {
//        backUltrasonic = ultrasonicSensor(backUltraS);
//        rightUltrasonic = ultrasonicSensor(rightUltraS);
//        leftUltrasonic = ultrasonicSensor(leftUltraS);
//        frontUltrasonic = ultrasonicSensor(frontUltraS);
    }

    //sets cam up
    public void setupOpenCV() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    //uses input from webcam and sets up video stream
    public void setupWebcamStream() throws InterruptedException {
        webcam.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                return input;
            }
        });

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });
    }

    //sets up stream that TFOD will use
    public void setupVuforia() throws InterruptedException {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = hardwareMap.get(WebcamName.class, webcamS);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    //uses Vuforia to recognize the object
    public void setupTFOD() throws InterruptedException {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.45f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(FFBCDM_MODEL_ASSET, FFBCDM_LABELS);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.65, 16.0/9.0);
        }
    }

    //-----------------------HARDWARE SETUP FUNCTIONS---------------------------------------

    //constructor to call in setup functions above for motors in drivetrain, intake, etc
    public DcMotor motor(String name, DcMotor.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setPower(0);
        return motor;
    }

    //servo constructor for linear slide servo set up
    public Servo servo(String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException {
        Servo servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }

    //constructor for servo with continuous rotation
    public CRServo servo(String name, DcMotorSimple.Direction direction, double startSpeed) throws InterruptedException {
        CRServo servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);

        servo.setPower(startSpeed);
        return servo;
    }

    //color sesnsor constructor
    public ColorSensor colorSensor(String name, boolean ledOn) throws InterruptedException {
        ColorSensor sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        central.telemetry.addData("Beacon Red Value: ", sensor.red());
        central.telemetry.update();

        return sensor;
    }

    //creates ultrasonic sensor
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

    //turns on encoder and sets up for conversions
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

    //IMU Utilities
    public double getYaw() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yawRaw = -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        //Check if delta raw readings is greater than threshold
        if (yawRaw - lastYawRaw > overflowThreshold)
        {
            //Detect and revert overflow
            yaw -= fullCircleDeg;
        }
        //Check if delta raw readings is less than negative threshold
        else if (yawRaw - lastYawRaw < -overflowThreshold)
        {
            //Detect and revert overflow
            yaw += fullCircleDeg;
        }

        //Add delta to current software sensor data
        yaw += yawRaw - lastYawRaw;

        //Save reading as old reading
        lastYawRaw = yawRaw;

        //return overflow protected yaw
        return yaw;
    }

    //Drivetrain Utilities

    //uses encoders and adds motors to drivetrain
    public void setDriveTrain(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motor);
                central.idle();
                setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER, motor);
                break;
            case OFF:
                break;
        }

        this.drivetrain = motor;
    }

    //sets the run mode of motor(s)
    public void setDriveTrainRunMode(DcMotor.RunMode runMode, DcMotor... motors) throws InterruptedException {
        for (DcMotor motor: motors) motor.setMode(runMode);
    }


    //sets up motors using the count ratio and making it usable

    //drivetrain uses 435 motors, calls encoder using 435 function with the set 435 ratio, moves it to specified distance with speed
    public void driveTrainEncoderMovement(double speed, double distance, movements movement) throws InterruptedException {
        driveTrainEncoderMovementSpecific435Motors(speed, distance, movement, drivetrain);
    }

    //for 435 motors, puts in 435 ratio into driveTrainEncoderMovementSpecificMotorsTypes and moves it a certain distance with speed
    public void driveTrainEncoderMovementSpecific435Motors(double speed, double distance, movements movement, DcMotor... motors) throws InterruptedException {
        driveTrainEncoderMovementSpecificMotorsTypes(speed, distance, movement, COUNTS_PER_INCH_GOBILDA_435_RPM, motors);
    }

    //for core hex motors, puts in core hex ratio into driveTrainEncoderMovementSpecificMotorsTypes and moves it a certain distance with speed
    public void driveTrainEncoderMovementSpecificCoreHexMotors(double speed, double distance, movements movement, DcMotor... motors) throws InterruptedException {
        driveTrainEncoderMovementSpecificMotorsTypes(speed, distance, movement, COUNTS_PER_INCH_REV_CORE_HEX_MOTOR, motors);
    }

    //allows different types of motors to move a specified distance with a speed, using motors' respective counts per inch ratio, and the direction specified through the movements
    public void driveTrainEncoderMovementSpecificMotorsTypes(double speed, double distance, movements movement, double COUNTS_PER_INCH_OF_MOTOR, DcMotor... motors) throws InterruptedException {
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                if (signs[x] != 0) motor.setTargetPosition(motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH_OF_MOTOR));
            }
            setDriveTrainRunMode(DcMotor.RunMode.RUN_TO_POSITION, motors);
            setMotorsPower(speed, movement, motors);

            // keep looping while we are still active and both motors are running.
            boolean drivetrainIsBusy = true;
            while (central.opModeIsActive() && drivetrainIsBusy) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();

                for (DcMotor motor: motors){
                    int i = Arrays.asList(motors).indexOf(motor);
                    if (!motor.isBusy() && signs[i] != 0) {
                        drivetrainIsBusy = false;
                    }
                }
            }

            // Stop all motion;
            setMotorsPower(0, motors);

            // Turn off RUN_TO_POSITION
            setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        }
    }

    //angles linear slide set degrees w set speed using its motor
    public void moveLinearSlideDegrees(double speed, double degrees, DcMotor linearSlide) {
        moveSingleMotorUnits(speed, degrees, COUNTS_PER_DEGREE_REV_CORE_HEX_MOTOR, linearSlide);
    }

    //extends linear slides set inches w set speed using motor
    public void moveLinearSlideInches(double speed, double inches, DcMotor linearSlide) {
        moveSingleMotorUnits(speed, inches, COUNTS_PER_INCH_LINEAR_SLIDE_MOTOR, linearSlide);
    }

    //function to use motor to move to a certain position
    public void moveSingleMotorUnits(double speed, double degrees, double COUNTS_PER_UNIT, DcMotor motor) {
        int sign = speed < 0 || degrees < 0 ? -1 : 1;
        motor.setTargetPosition(motor.getCurrentPosition() + (int) (sign * Math.abs(degrees) * COUNTS_PER_UNIT));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(sign * Math.abs(speed));
        while (motor.isBusy());
        motor.setPower(0);
    }

    //runs intake motor at certain speed for set time
    public void runIntakeTimeSpeed(double speed, long time) {
        runSingleMotorTimeSpeed(speed, time, intake, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //runs intake motor without set time but w set speed
    public void runIntakeSpeed(double speed) {
        runSingleMotorSpeed(speed, intake, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //turns on left carousel motor at certain speed for set time
    public void runCarouselsTimeSpeed(double speed, long time) {
        runCarouselsSpeed(speed);
        central.sleep(time);
        runCarouselsSpeed(0);
    }

    //turns on left carousel motor without set time but w set speed
    public void runCarouselsSpeed(double speed) {
        runSingleMotorSpeed(speed, carouselRight, DcMotor.RunMode.RUN_USING_ENCODER);
        runSingleMotorSpeed(speed, carouselLeft, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //moves linearside motor at a speed
    public void moveLinearSlide(double speed, DcMotor linearSlide) {
        runSingleMotorSpeed(speed, linearSlide, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //runs motor specified motor at set speed and time
    public void runSingleMotorTimeSpeed(double speed, long time, DcMotor motor, DcMotor.RunMode runMode) {
        runSingleMotorSpeed(speed, motor, runMode);
        central.sleep(time);
        motor.setPower(0);
    }

    //runs motor at a certain speed without time
    public void runSingleMotorSpeed(double speed, DcMotor motor, DcMotor.RunMode runMode) {
        motor.setMode(runMode);
        motor.setPower(speed);
    }

    //------------------DRIVETRAIN TELEOP FUNCTIONS------------------------------------------------------------------------
    //moves drivetrain, setting each motor to its right power
    public void driveTrainMovement(double speed, double[] signs, movements movement) {
        for (int i = 0; i < signs.length; i++) {
            signs[i] *= movement.getDirections()[i];
        }
        driveTrainMovement(speed, signs);
    }

    public void driveTrainMovement(double speed, movements movement) {
        setMotorsPower(speed, movement, drivetrain);
    }

    public void driveTrainMovement(double speed, double[] signs) {
        setMotorsPower(speed, signs, drivetrain);
    }

    public void setMotorsPower(double speed, movements movement, DcMotor... motors) {
        double[] signs = movement.getDirections();
        setMotorsPower(speed, signs, motors);
    }

    public void setMotorsPower(double speed, double[] signs, DcMotor... motors) {
        for (DcMotor motor: motors){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            if (signs[x] != 0) motor.setPower(signs[x] * Math.abs(speed));
        }
    }

    public void setMotorsPower(double speed, DcMotor... motors) { for (DcMotor motor : motors) motor.setPower(speed); }

    //sets each motor power to 0, stops it
    public void stopDrivetrain() throws InterruptedException {
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }

    // IMU Movements

    //turns motors a number of degrees, accounts for possible offset and rotation axis based on which motors are used
    public void turn(double speed, double degrees) throws InterruptedException { turn(speed, degrees, axis.center); }

    public void turn(double speed, double degrees, axis rotationAxis) throws InterruptedException {
        double startAngle = getYaw();

        switch (rotationAxis) {
            case center:
                driveTrainMovement(speed, degrees > 0 ? movements.cw : movements.ccw);
                break;
            case back:
                driveTrainMovement(speed, degrees > 0 ? movements.cwback : movements.ccwback);
                break;
            case front:
                driveTrainMovement(speed, degrees > 0 ? movements.cwfront : movements.ccwfront);
                break;
        }

        if (Math.abs(degrees) > Math.max(turnOffsetPositive, turnOffsetNegative)) while (central.opModeIsActive() && degrees > 0 ? getYaw() - startAngle < degrees - turnOffsetPositive : getYaw() - startAngle > degrees + turnOffsetNegative);
        else while (central.opModeIsActive() && degrees > 0 ? getYaw() - startAngle < degrees : getYaw() - startAngle > degrees);

        stopDrivetrain();
    }

    public double getDirection(){
        return (this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle-initorient+720)%360;
    }

    public enum EncoderMode{
        ON, OFF;
    }

    public enum setupType{
        autonomous, teleop, drivetrain_system, ultra, intake, crane, carousel, imu, openCV, webcamStream, vuforia, tfod;
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
        forward(1, -1, 1, -1),
        backward(-1, 1, -1, 1),
        left(1, 1, -1, -1),
        right(-1, -1, 1, 1),
        br(0, -1, 1, 0),
        bl(1, 0, 0, -1),
        tl(0, 1, -1, 0),
        tr(-1, 0, 0, 1),
        cw(-1, -1, -1, -1),
        ccw(1, 1, 1, 1),
        cwfront(-1, -1, 0, 0),
        ccwfront(1, 1, 0, 0),
        cwback(0, 0, -1, -1),
        ccwback(0, 0, 1, 1);

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

    public enum axis {
        front, center, back
    }

    public static double quadrantAtan(double x, double y) {
        return Math.atan(y/x) + (x < 0 ? Math.PI : 0) + (x >= 0 && y < 0 ? 2 * Math.PI : 0);
    }

    /**
     * Finds the set of two direction speeds at the mecanum's movement angles that create a vector movement towards the desired angle
     * @param speed speed of movement
     * @param angleDegrees angle (in degrees) of movement
     * @return array of motor directions
     */
    public static double[] anyDirection(double speed, double angleDegrees) {
        return anyDirection(speed, angleDegrees, 0);
    }

    public static double[] anyDirection(double speed, double angleDegrees, double rotationOffsetDegrees) {
        return anyDirectionRadians(speed, Math.toRadians(angleDegrees), Math.toRadians(rotationOffsetDegrees));
    }

    //same as above, uses param angleRadians
    public static double[] anyDirectionRadians(double speed, double angleRadians) {
        return anyDirectionRadians(speed, angleRadians, 0);
    }

    public static double[] anyDirectionRadians(double speed, double angleRadians, double rotationOffsetRadians) {
        double beta = Math.atan(yToXRatio);

        double v1 = Math.sin(angleRadians - (beta + rotationOffsetRadians));
        double v2 = Math.cos(angleRadians - (beta + rotationOffsetRadians));

        double v1Max = speed * (v1 / Math.max(Math.abs(v1), Math.abs(v2)));
        double v2Max = speed * (v2 / Math.max(Math.abs(v1), Math.abs(v2)));

        return new double[] {v1Max, v2Max};
    }

    public void driveTrainMovementAngle(double speed, double angleDegrees) {
        driveTrainMovementAngle(speed, angleDegrees, 0);
    }

    public void driveTrainMovementAngle(double speed, double angleDegrees, double rotationOffsetDegrees) {
        double[] speeds = anyDirection(speed, angleDegrees, rotationOffsetDegrees);
        driveTrainMovement(speed, new double[] {speeds[0], speeds[1], speeds[1], speeds[0]}, movements.forward);
    }

    public void driveTrainMovementAngleRadians(double speed, double angleRadians) {
        driveTrainMovementAngleRadians(speed, angleRadians, 0);
    }

    public void driveTrainMovementAngleRadians(double speed, double angleRadians, double rotationOffsetRadians) {
        double[] speeds = anyDirectionRadians(speed, angleRadians, rotationOffsetRadians);
        driveTrainMovement(speed, new double[] {speeds[0], speeds[1], speeds[1], speeds[0]}, movements.forward);
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
