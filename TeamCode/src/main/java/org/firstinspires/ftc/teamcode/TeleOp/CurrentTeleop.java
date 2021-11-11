package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name="CurrentTeleop", group = "basic")
public class CurrentTeleop extends TeleOpControl
{
    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.teleop);

        waitForStart();

        while (opModeIsActive()) {
            standardGamepadData();

            if (g(0)) {
                rob.driveTrainMovement(fb, Goal.movements.forward);
            } else if (g(2)) {
                rob.driveTrainMovement(fb, Goal.movements.backward);
            } else if (g(3)) {
                rob.driveTrainMovement(rl, Goal.movements.right);
            } else if (g(1)) {
                rob.driveTrainMovement(rl, Goal.movements.left);
            } else if (g(4)) {
                rob.driveTrainMovement(diagonalSpeed, Goal.movements.br);
            }else if (g(5)) {
                rob.driveTrainMovement(diagonalSpeed, Goal.movements.bl);
            }else if (g(6)) {
                rob.driveTrainMovement(diagonalSpeed, Goal.movements.tl);
            }else if (g(7)) {
                rob.driveTrainMovement(diagonalSpeed, Goal.movements.tr);
            } else if (g(8)) {
                rob.driveTrainMovement(1, Goal.movements.ccw);
            } else if (g(9)) {
                rob.driveTrainMovement(1, Goal.movements.cw);
            } else {
                rob.stopDrivetrain();
            }
        }
    }
}