package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="Good Ultrasonic Test", group = "basic")
public class TestUltrasonic extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.ultra);
        telemetry.addLine("Start!");
        telemetry.update();

        while (opModeIsActive()) {
            //telemetry.addData("raw ultrasonic front", rob.front.rawUltrasonic());
            //telemetry.addData("raw optical front", rob.front.rawOptical());
            //telemetry.addData("cm optical front", "%.2f cm", rob.front.cmOptical());
            telemetry.addData("Inches front", "%.2f in", rob.Right.getDistance(DistanceUnit.INCH));

            //telemetry.addData("raw ultrasonic back", rob.back.rawUltrasonic());
            //telemetry.addData("raw optical back", rob.back.rawOptical());
            //telemetry.addData("cm optical back", "%.2f cm", rob.back.cmOptical());
            telemetry.addData("Inches back", "%.2f in", rob.Right.getDistance(DistanceUnit.INCH));

            //telemetry.addData("raw ultrasonic left", rob.left.rawUltrasonic());
            //telemetry.addData("raw optical left", rob.left.rawOptical());
            //telemetry.addData("cm optical left", "%.2f cm", rob.left.cmOptical());

            //telemetry.addData("raw ultrasonic right", rob.right.rawUltrasonic());
            //telemetry.addData("raw optical right", rob.right.rawOptical());
            //telemetry.addData("cm optical right", "%.2f cm", rob.right.cmOptical());
            telemetry.addData("Inches right", "%.2f in", rob.Back.getDistance(DistanceUnit.INCH));

            telemetry.update();
        }


    }
}
