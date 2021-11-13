package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.Goal.movements;

import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_DEGREE_REV_CORE_HEX_MOTOR;

@Autonomous(name="RedAuton", group="basic")
public class RedAuton extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.autonomous);

        double speed = 0.5;
        long timeoutS = 10000;
        long waitAfter = 0;

        boolean doingCarousel = false;
        if (doingCarousel) {
            rob.driveTrainEncoderMovement(speed, 6, timeoutS, waitAfter, movements.right);
            rob.driveTrainEncoderMovement(speed, 12, timeoutS, waitAfter, movements.forward);
            rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, movements.cw);
            rob.runCarouselTimeSpeed(-1, 1500);
            rob.driveTrainEncoderMovement(speed, 12, timeoutS, waitAfter, movements.left);
            rob.driveTrainEncoderMovement(speed, 6, timeoutS, waitAfter, movements.cw);
            rob.driveTrainEncoderMovement(speed, 48, timeoutS, waitAfter, movements.left);
        }
        else {
            rob.driveTrainEncoderMovement(speed, 4, timeoutS, waitAfter, movements.backward);
        }

        rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, movements.backward);
        rob.rotate.setPosition(0.5);
        sleep(500);
        rob.claw.setPosition(0.2);
        sleep(500);
        rob.rotate.setPosition(0);
        rob.claw.setPosition(0.05);
        rob.driveTrainEncoderMovement(speed, 20, timeoutS, waitAfter, movements.forward);
        rob.driveTrainEncoderMovement(speed, 28, timeoutS, waitAfter, movements.cw);
        rob.driveTrainEncoderMovement(speed/2, 84, timeoutS, waitAfter, movements.forward);
        rob.driveTrainEncoderMovement(speed, 28, timeoutS, waitAfter, movements.left);
        rob.driveTrainEncoderMovement(speed, 24, timeoutS, waitAfter, movements.forward);
    }
}