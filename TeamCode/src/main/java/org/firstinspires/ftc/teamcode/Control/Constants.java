package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Constants {
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
         * @param counts_per_rev    Counts per revolution of specified enum element
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

    /**
     * Enum of diameter in inches by object with circumference (wheels), add new circumference objects here.
     */
    public enum circumferenceObject{
        GO_BILDA_STANDARD_MECANUM(4),
        GREEN_COMPLIANT_WHEEL(4);

        /**
         * Diameter in inches
         */
        double diameter;

        /**
         * @param diameterInches    Diameter in inches of specific enum element
         */
        circumferenceObject(double diameterInches) {
            this.diameter = diameterInches;
        }
    }

    /**
     * Conversion factor of inch to mm since Vuforia uses mm, so mm must be used for all physical dimensions
     */
    public static final float mmPerInch = 25.4f;

    /**
     * Counts per revolution of a GoBilda 117 RPM Motor
     */
    public static final double COUNTS_PER_MOTOR_GOBILDA_117_RPM = 1425.1;

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

    /**
     * Counts per revolution of a GoBilda 30 RPM Motor
     */
    public static final double COUNTS_PER_MOTOR_GOBILDA_30_RPM = 5281.1;

    /**
     * Drive Gear Reduction of a GoBilda 312 RPM Motor
     */
    public static final double DRIVE_GEAR_REDUCTION_GOBILDA_312_RPM = 2.0/3.0;

    /**
     * Drive Gear Reduction of a GoBilda 435 RPM Motor
     */
    public static final double DRIVE_GEAR_REDUCTION_GOBILDA_435_RPM = 2.0;

    /**
     * Drive Gear Reduction of a REV Core HEX Motor
     */
    public static final double DRIVE_GEAR_REDUCTION_REV_CORE_HEX_MOTOR = 1.0;

    /**
     * Wheel Diameter of the wheels on our GoBilda 312 RPM Motors
     */
    public static final double WHEEL_DIAMETER_INCHES_GOBILDA_312_RPM = 4.0;

    /**
     * Wheel Diameter of the wheels on our GoBilda 435 RPM Motors
     */
    public static final double WHEEL_DIAMETER_INCHES_GOBILDA_435_RPM = 96 / mmPerInch;

    /**
     * Wheel Diameter of the wheels on our REV Core HEX Motors
     */
    public static final double WHEEL_DIAMETER_INCHES_REV_CORE_HEX_MOTOR = 1.25;

    /**
     * Wheel Diameter of the wheel on the REV Core HEX Motor used on the linear slide
     */
    public static final double WHEEL_DIAMETER_MM_LINEAR_SLIDE_MOTOR = 31.75;

    /**
     * WHEEL_DIAMETER_MM_LINEAR_SLIDE_MOTOR converted to inches
     */
    public static final double WHEEL_DIAMETER_INCHES_LINEAR_SLIDE_MOTOR = WHEEL_DIAMETER_MM_LINEAR_SLIDE_MOTOR / mmPerInch;

    /**
     * Counts per inch calculated of a GoBilda 312 RPM Motor
     */
    public static final double COUNTS_PER_INCH_GOBILDA_312_RPM = (COUNTS_PER_MOTOR_GOBILDA_312_RPM * DRIVE_GEAR_REDUCTION_GOBILDA_312_RPM) /
            (WHEEL_DIAMETER_INCHES_GOBILDA_312_RPM * Math.PI);

    /**
     * Counts per inch calculated of a GoBilda 435 RPM Motor
     */
    public static final double COUNTS_PER_INCH_GOBILDA_435_RPM = COUNTS_PER_MOTOR_GOBILDA_435_RPM /
            (WHEEL_DIAMETER_INCHES_GOBILDA_435_RPM * Math.PI);

    /**
     * Counts per inch calculated of a REV Core HEX Motor
     */
    public static final double COUNTS_PER_INCH_REV_CORE_HEX_MOTOR = (COUNTS_PER_MOTOR_REV_CORE_HEX_MOTOR * DRIVE_GEAR_REDUCTION_REV_CORE_HEX_MOTOR) /
            (WHEEL_DIAMETER_INCHES_REV_CORE_HEX_MOTOR * Math.PI);

    /**
     * Counts per inch calculated for linear slide REV Core HEX Motor
     */
    public static final double COUNTS_PER_INCH_LINEAR_SLIDE_MOTOR = COUNTS_PER_MOTOR_REV_CORE_HEX_MOTOR / (WHEEL_DIAMETER_INCHES_LINEAR_SLIDE_MOTOR * Math.PI);

    /**
     * Counts per degree calculated of REV Core HEX Motor
     */
    public static final double COUNTS_PER_DEGREE_REV_CORE_HEX_MOTOR = COUNTS_PER_MOTOR_REV_CORE_HEX_MOTOR / 360.0;

    /**
     * Counts per degree calculated of GoBilda 117 RPM Motor
     */
    public static final double COUNTS_PER_DEGREE_GOBILDA_117_RPM = COUNTS_PER_MOTOR_GOBILDA_117_RPM / 360.0;

    /**
     * Counts per degree calculated of GoBilda 312 RPM Motor
     */
    public static final double COUNTS_PER_DEGREE_GOBILDA_312_RPM = COUNTS_PER_MOTOR_GOBILDA_312_RPM / 360.0;

    //--------------------------------TELE-OP VALUES--------------------
    /**
     * Joystick Dead Zone Threshold
     */
    public static final double DEAD_ZONE_SIZE = 0.05;

    //--------------------------------VUFORIA----------------------------
    /**
     * Generated Key for Vuforia use
     */
    public static final String VUFORIA_KEY = "ASYtBET/////AAABmSTQiLUzLEx3qLnHm6hu7Y1aNDWPDgMBKY8lFonYrzU8M5f9mAV5KiaJ9YZWCSgoUx6/AKuobb1cLgB8R+mDHgx6FoP3XS3K8bAwShz98sojuAKmTGzJMZVUjH8mjW+9ebYjtw3oZr/ZM2F2NZuCPN4Rx+K5koMfR2IE1OQKoZbkgLJSc36yUmis7MN91L0xIgntCKhqpZkRX45VjWsZi4BcKQnK5L2YfUqueZ7qvPzpF7sWDDcWYqkLZNbxfRk+gUVdabq/uOPYR8v0O0EFONv7h2kiU3E1s7Rm8WOukfwfqa5Nsw7FSNF2kjL0PhPbGPBQ6kVbLQMsvmxM7x/AA2owHe8l1yHgzyCgd7YTFOdi";

    /**
     * Generated Key for Tensorflow use
     */
    public static final String TENSORFLOW_KEY = "Adb83BH/////AAABmTheak2ntU3VnH1pRcX2UDVJc60lqKXP9o54kAOKZoMvggLhrVVWOQ06E0yXEF3xRwJADjy5U2N519egNSjJ+Kj6jr05a6UmqLTEXS8elc2jYhx+T5P0pbc3ByKBdqw0lwBzL15jcqFrNDmbTH5hsuZjRP0RfvE1k/S2VW3wvD8U3GNtd2wb7xdQbmysXoDrNk0s+bgyn4mCX8jNL33RvYuIYfDKkC215c+jbYjn4rDAHNyM02Va777s5mcbYTb3LAX0iVYQApbtX4MjcPyU+D5p5dRQVYTE2hVtbMVvJg66m7ZcZ8aRV1GwTEYYVhq6z/iT3+cDH2pjNXtb0mGwHwyAnCwSMVqFtpbQ4DrC/3uj";

    /**
     * The width of the FTC field from one side to the other
     */
    public static final float mmFTCFieldWidth = (12/*ft*/ * 12/*in*/) * mmPerInch;

    /**
     * Half of the width of the FTC field
     */
    public static final float mmFTCHalfFieldWidth = mmFTCFieldWidth/2f;

    /**
     * Quarter of the width of the FTC field
     */
    public static final float mmFTCQuadFieldWidth = mmFTCFieldWidth/4f;

    /**
     * The height of the ImageTarget (from the image center to the floor)
     */
    public static final float mmTargetHeight = (6/*in*/) * mmPerInch;

    /**
     * Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
     * Valid choices are:  BACK or FRONT
     */
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    /**
     * Orientation of phone used for Vuforia
      */
    public static final boolean PHONE_IS_PORTRAIT = false;

    //--------------------------------CONFIGURATION VALUES--------------------
    /**
     * Front right motor name
     */
    public static final String motorFRS = "motorFR";

    /**
     * Front left motor name
     */
    public static final String motorFLS = "motorFL";

    /**
     * Back right motor name
     */
    public static final String motorBRS = "motorBR";

    /**
     * Back left motor name
     */
    public static final String motorBLS = "motorBL";

    /**
     * IMU name
     */
    public static final String imuS = "imu";

    /**
     * Ultrasonic names
     */
    public static final String backUltraS = "Back";
    public static final String rightUltraS = "Right";
    public static final String leftUltraS = "Left";
    public static final String frontUltraS = "Front";

    /**
     * Intake motor name (435)
     */
    public static final String intakeS = "intake";

    /**
     * Crane Lift motor name (435)
     */
    public static final String craneLiftS = "craneLift";

    /**
     * Crane Pivot motor name (312)
     */
    public static final String cranePivotS = "cranePivot";

    /**
     * Crane Bucket Left servo name
     */
    public static final String bucketLeftS = "bucketLeft";

    /**
     * Crane Bucket Right servo name
     */
    public static final String bucketRightS = "bucketRight";

    /**
     * Carousel motor name (312)
     */
    public static final String carouselS = "carousel";

    /**
     * Webcam name
     */
    public static final String webcamS = "Webcam 1";

    /**
     * FTC Provided Freight Frenzy Model Name
     */
    public static final String FFBCDM_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

    /**
     * Labels for FFBCDM_MODEL_ASSET
     */
    public static final String[] FFBCDM_LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /**
     * Team Element Model Name
     */
    public static final String FTEDD_MODEL_ASSET = "FTEDDv1.tflite";

    /**
     * Labels for FTEDD_MODEL_ASSET
     */
    public static final String[] FTEDD_LABELS = {
            "middle",
            "right"
    };
}
