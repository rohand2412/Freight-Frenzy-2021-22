package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Constants {

//hi
    //--------------------------------ENCODERS-------------------------

    /**
     * Enum of encoder counts by motor, add new motor counts in here.
     * Includes instance function for countsPerInch()
     */
    public enum encoderCounts{
        GOBILDA_312(537.6),
        GOBILDA_435(383.6),
        REV_CORE_HEX(288),
        REV_STANDARD_MOTOR(1120),
        ;

        /**
         * Counts per revolution of specified enum value
         */
        double counts_per_rev;

        /**
         * @param counts_per_rev    Counts per revolution of specified enum value
         */
        encoderCounts(double counts_per_rev) {
            this.counts_per_rev = counts_per_rev;
        }

        /**
         * Returns counts per inch for the motor assuming no gearing.
         * @param object wheelType of {@link circumferenceObject} enum
         * @return counts per inch
         */
        public double countsPerInch(circumferenceObject object){
            return countsPerInch(1, object.diameter);
        }

        /**
         * Returns counts per inch for the motor assuming no gearing.
         * @param diameterInches diameter (in inches)
         * @return counts per inch
         */
        public double countsPerInch(double diameterInches){
            return countsPerInch(1, diameterInches);
        }

        /**
         * Returns counts per inch for the motor.
         * @param drive_reduction gearing ratio, > 1 if geared for speed
         * @return counts per inch
         */
        public double countsPerInch(double drive_reduction, double diameterInches){
            return this.counts_per_rev * drive_reduction / (diameterInches * Math.PI);
        }

    }

    public enum circumferenceObject{
        GO_BILDA_STANDARD_MECANUM(4),
        GREEN_COMPLIANT_WHEEL(4);

        double diameter;

        circumferenceObject(double diameterInches) {
            this.diameter = diameterInches;
        }
    }

    /**
     * Counts per revolut
     */
    public static final double COUNTS_PER_MOTOR_REV = 1120;
    public static final double COUNTS_PER_MOTOR_TETRIX = 1440;

    /**
     * Counts per revolution of a GoBilda 312 RPM Motor
     */
    public static final double COUNTS_PER_MOTOR_GOBILDA_312_RPM = 537.6;

    /**
     * Counts per revolution of a GoBilda 435 RPM Motor
     */
    public static final double COUNTS_PER_MOTOR_GOBILDA_435_RPM = 383.6;

    /**
     * Counts per revolution of a REV Core HEX Motor
     */
    public static final double COUNTS_PER_MOTOR_REV_CORE_HEX_MOTOR = 288;

    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double DRIVE_GEAR_REDUCTION_NEW = 2.0/3.0;
    public static final double DRIVE_GEAR_REDUCTION_EVENNEWER = 2.0;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double WHEEL_DIAMETER_INCHES_NEW = 3.94;
    public static final double SPOOL_DIAMETER_INCHES = 1.25;

    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches


    public static final double COUNTS_PER_TETRIX_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);               // Conversion: Encoder Counts Motor Tetrix to Inches


    public static final double COUNTS_PER_GOBUILDA312RPM_INCH = (COUNTS_PER_MOTOR_GOBILDA_312_RPM * DRIVE_GEAR_REDUCTION_NEW) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    public static final double COUNTS_PER_GOBUILDA435RPM_INCH = (COUNTS_PER_MOTOR_GOBILDA_435_RPM * DRIVE_GEAR_REDUCTION_EVENNEWER) /
            (WHEEL_DIAMETER_INCHES_NEW * 3.1415);


    public static final double COUNTS_PER_COREHEXMOTOR_INCH = (COUNTS_PER_MOTOR_REV_CORE_HEX_MOTOR * DRIVE_GEAR_REDUCTION) /
            (SPOOL_DIAMETER_INCHES * 3.1415);
    public static final double COUNTS_PER_GOBUILDA312RPM_ROT = (COUNTS_PER_MOTOR_GOBILDA_312_RPM * DRIVE_GEAR_REDUCTION_NEW);

    //--------------------------------TELE-OP VALUES--------------------
    public static final double DEAD_ZONE_SIZE = 0.1;

    //--------------------------------VUFORIA----------------------------

    public static final String VUFORIA_KEY = "ASYtBET/////AAABmSTQiLUzLEx3qLnHm6hu7Y1aNDWPDgMBKY8lFonYrzU8M5f9mAV5KiaJ9YZWCSgoUx6/AKuobb1cLgB8R+mDHgx6FoP3XS3K8bAwShz98sojuAKmTGzJMZVUjH8mjW+9ebYjtw3oZr/ZM2F2NZuCPN4Rx+K5koMfR2IE1OQKoZbkgLJSc36yUmis7MN91L0xIgntCKhqpZkRX45VjWsZi4BcKQnK5L2YfUqueZ7qvPzpF7sWDDcWYqkLZNbxfRk+gUVdabq/uOPYR8v0O0EFONv7h2kiU3E1s7Rm8WOukfwfqa5Nsw7FSNF2kjL0PhPbGPBQ6kVbLQMsvmxM7x/AA2owHe8l1yHgzyCgd7YTFOdi";

    public static final String TENSORFLOW_KEY = "Adb83BH/////AAABmTheak2ntU3VnH1pRcX2UDVJc60lqKXP9o54kAOKZoMvggLhrVVWOQ06E0yXEF3xRwJADjy5U2N519egNSjJ+Kj6jr05a6UmqLTEXS8elc2jYhx+T5P0pbc3ByKBdqw0lwBzL15jcqFrNDmbTH5hsuZjRP0RfvE1k/S2VW3wvD8U3GNtd2wb7xdQbmysXoDrNk0s+bgyn4mCX8jNL33RvYuIYfDKkC215c+jbYjn4rDAHNyM02Va777s5mcbYTb3LAX0iVYQApbtX4MjcPyU+D5p5dRQVYTE2hVtbMVvJg66m7ZcZ8aRV1GwTEYYVhq6z/iT3+cDH2pjNXtb0mGwHwyAnCwSMVqFtpbQ4DrC/3uj";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


    //--------------------------------CONFIGURATION VALUES--------------------

    public static final String motorFRS = "motorFR";
    public static final String motorFLS = "motorFL";
    public static final String motorBRS = "motorBR";
    public static final String motorBLS = "motorBL";
    public static final String flys = "fly";
    public static final String collections = "collection";
    public static final String shooterLeftS = "shooterLeft";
    public static final String shooterRightS = "shooterRight";

    public static final String feederLeftS = "feederLeft";
    public static final String feederRightS = "feederRight";

    public static final String claws = "claw";
   // public static final String grabbers = "grabber";M
    public static final String lifters = "lifter";

    public static final String pincher = "pinch";
    public static final String whacker = "whacker";
//    //public static final String ultraFrontS = "ultraFront";
//    public static final String leftSenseS = "leftSense";
//   // public static final String frontSenseS = "frontSense";
//    public static final String rightfrontSenseS = "rightfrontSense";
//    public static final String rightbackSenseS = "rightbackSense";
//    public static final String backSenseS = "backSense";

    public static final String imuS = "imu";

    public static final String leftFronts = "leftFront";
    public static final String Backs = "Back";
//    public static final String rightFronts = "rightFront";
//    public static final String rightBacks = "rightBack";
    public static final String Rights = "Right";
    public static final String Lefts = "Left";
    public static final String Fronts = "Front";
    public static final String colorSensorS = "color";
}
