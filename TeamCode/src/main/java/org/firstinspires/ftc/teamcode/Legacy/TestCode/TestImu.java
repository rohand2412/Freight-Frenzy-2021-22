package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name = "TestImu", group = "basic")
public class TestImu extends AutonomousControl
{
    @Override
    public void runOpMode() throws  InterruptedException
    {
        setup(runtime, Goal.setupType.imu);

        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addLine(rob.getYaw() + "");
            telemetry.update();
        }
    }
}
