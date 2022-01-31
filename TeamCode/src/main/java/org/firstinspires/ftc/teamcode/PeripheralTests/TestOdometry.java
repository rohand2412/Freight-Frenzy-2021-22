package org.firstinspires.ftc.teamcode.PeripheralTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.Robot;
import org.firstinspires.ftc.teamcode.Control._Autonomous;

@Autonomous(name="TestOdometry", group="PeripheralTest")
public class TestOdometry extends _Autonomous {

    @Override
    public void init() {
        Robot.setup(hardwareMap, telemetry, Robot.SetupType.Carousel, Robot.SetupType.CraneLift, Robot.SetupType.Intake);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        telemetry.addLine("Left: " + Robot.getCarousel().getCounts());
        telemetry.addLine("Middle: " + Robot.getCraneLift().getCounts());
        telemetry.addLine("Right: " + Robot.getIntake().getCounts());
    }

    @Override
    public void stop() {}
}