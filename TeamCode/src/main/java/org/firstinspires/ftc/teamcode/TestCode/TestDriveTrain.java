package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_DEGREE_REV_CORE_HEX_MOTOR;

@Autonomous(name="TestDriveTrain", group="basic")
public class TestDriveTrain extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.autonomous);

        //Whole drivetrain forward
        rob.driveTrainEncoderMovement(0.7, 12, 10000, 0, Goal.movements.forward);

        //Just back right motor forward
//        rob.driveTrainEncoderMovementSpecific435Motors(0.7, 12, 10000, 0, Goal.movements.forward, rob.motorBR);

        //Turn pivot motor 30 degrees at 0.1 speed
//        rob.pivot.setTargetPosition(rob.pivot.getCurrentPosition() + (int) (30.0 * COUNTS_PER_DEGREE_REV_CORE_HEX_MOTOR));
//        rob.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rob.pivot.setPower(0.1);
//        while (rob.pivot.isBusy());
//        rob.pivot.setPower(0);
//        sleep(5000);
    }
}