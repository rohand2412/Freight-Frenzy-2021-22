package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Control.Constants.DEAD_ZONE_SIZE;

public abstract class TeleOpControl extends Central {

    public static float yAxis1;
    public static float xAxis1;

    public static float yAxis2;
    public static float xAxis2;

    public static float fb;
    public static float rl;

    public static float fb2;
    public static float rl2;

    public static float yAxis3;
    public static float xAxis3;

    public static float yAxis4;
    public static float xAxis4;

    public static float fb3;
    public static float rl3;

    public static float fb4;
    public static float rl4;

    public static double diagonalSpeed;

    public static boolean rightStickButtonPressed;
    public static boolean leftStickButtonPressed;


    public void standardGamepadData(){

        yAxis1 = -gamepad1.left_stick_y; // Main Directions y-axis
        xAxis1 = gamepad1.left_stick_x;  // Main Directions x-axis

        yAxis2 = -gamepad1.right_stick_y; // Diagonal Directions y-axis
        xAxis2 = gamepad1.right_stick_x;  // Diagonal Directions x-axis

        yAxis1 = Range.clip(yAxis1, -1, 1);
        xAxis1 = Range.clip(xAxis1, -1, 1);

        yAxis2 = Range.clip(yAxis2, -1, 1);
        xAxis2 = Range.clip(xAxis2, -1, 1);

        fb = Math.abs(yAxis1);
        rl = Math.abs(xAxis1);

        fb2 = Math.abs(yAxis2);
        rl2 = Math.abs(xAxis2);
        diagonalSpeed = Math.hypot(xAxis2, yAxis2)/2;

        yAxis3 = -gamepad2.right_stick_y; // Main Directions y-axis
        xAxis3 = gamepad2.right_stick_x;  // Main Directions x-axis

        yAxis4 = -gamepad1.left_stick_y; // Diagonal Directions y-axis
        xAxis4 = gamepad1.left_stick_x;  // Diagonal Directions x-axis

        yAxis3 = Range.clip(yAxis3, -1, 1);
        xAxis3 = Range.clip(xAxis3, -1, 1);

        yAxis4 = Range.clip(yAxis4, -1, 1);
        xAxis4 = Range.clip(xAxis4, -1, 1);

        fb3 = Math.abs(yAxis3);
        rl3 = Math.abs(xAxis3);

        fb4 = Math.abs(yAxis4);
        rl4 = Math.abs(xAxis4);

        rightStickButtonPressed = gamepad1.right_stick_button;
        leftStickButtonPressed = gamepad1.left_stick_button;
    }

    public boolean validStick(double x, double y){
        return Math.pow(x, 2) + Math.pow(y, 2) >= Math.pow(DEAD_ZONE_SIZE, 2);
    }
    //Jittery
    //----------------EXPERIMENTAL RECONSTRUCTION ------------------------




    /*
    left_stick       right_stick
    -       -             |
     -  0  -          5   |   4         left_button      8
       - -         -------|-------      right_button     9
   1   - -   3            |
     -     -          6   |   7
    -   2   -             |
     */


    public boolean g(int n){ // Returns whether

        return new boolean[]{yAxis1 >= Math.abs(xAxis1), -Math.abs(yAxis1) > xAxis1, yAxis1 <= -Math.abs(xAxis1), Math.abs(yAxis1) < xAxis1,
                yAxis2 >= 0 && xAxis2 >= 0, yAxis2 >= 0 && xAxis2 < 0, yAxis2 < 0 && xAxis2 < 0, yAxis2 < 0 && xAxis2 >= 0,
                gamepad1.left_stick_button, gamepad1.right_stick_button}[n]

                && (n < 4 ? (validStick(xAxis1, yAxis1)) : (n >= 8 || validStick(xAxis2, yAxis2)));

    }
    public boolean f(int n){ // Returns whether

        return new boolean[]{yAxis3 >= Math.abs(xAxis3), -Math.abs(yAxis3) > xAxis3, yAxis3 <= -Math.abs(xAxis3), Math.abs(yAxis3) < xAxis3,
                yAxis4 >= 0 && xAxis4 >= 0, yAxis4 >= 0 && xAxis4 < 0, yAxis4 < 0 && xAxis4 < 0, yAxis4 < 0 && xAxis4 >= 0,
                gamepad2.left_stick_button, gamepad2.right_stick_button}[n]

                && (n < 4 ? (validStick(xAxis3, yAxis3)) : (n >= 8 || validStick(xAxis4, yAxis4)));

    }


    public void setMotorPower(double power, DcMotor... motors){
        for (DcMotor b :
                motors) {
            b.setPower(power);
        }
    }

}
