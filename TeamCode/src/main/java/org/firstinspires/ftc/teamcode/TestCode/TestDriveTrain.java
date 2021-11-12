package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.Goal.movements;

import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_DEGREE_REV_CORE_HEX_MOTOR;

@Autonomous(name="TestDriveTrain", group="basic")
public class TestDriveTrain extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.autonomous);

        //Test each motor individually (FR, FL, BR, BL)
        for (DcMotor motor : rob.drivetrain) {
            rob.driveTrainEncoderMovementSpecific435Motors(0.7, 12, 10000, 0, Goal.movements.forward, motor);
            sleep(1000);
        }

        //Test each movement individually
        for (movements movement : movements.values()) {
            rob.driveTrainEncoderMovement(0.7, 12, 10000, 0, movement);
            sleep(1000);
        }
    }
}