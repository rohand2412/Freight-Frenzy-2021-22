package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="TestUltra", group="basic")
public class TestUltra extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.ultra);

        while (opModeIsActive()) {
            telemetry.addLine("Front: " + rob.frontUltrasonic.getDistance(DistanceUnit.CM) + "");
            telemetry.addLine("Right: " + rob.rightUltrasonic.getDistance(DistanceUnit.CM) + "");
            telemetry.addLine("Left: " + rob.leftUltrasonic.getDistance(DistanceUnit.CM) + "");
            telemetry.update();
        }
    }
}