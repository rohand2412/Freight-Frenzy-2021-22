package org.firstinspires.ftc.teamcode.PeripheralTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.Robot;
import org.firstinspires.ftc.teamcode.Control._Autonomous;
import org.firstinspires.ftc.teamcode.Drivers._IMU;

@Autonomous(name="CraneIMUTest", group="PeripheralTest")
public class CraneIMUTest extends _Autonomous {

    private _IMU _imu;

    @Override
    public void init() {
        Robot.setup(hardwareMap, telemetry);
        _imu = new _IMU("armImu", true, false);
    }

    @Override
    public void init_loop() {
        _imu.update();
        telemetry.addLine(_imu.getName());
        telemetry.addLine(String.valueOf(_imu.getYaw()));
        telemetry.addLine(String.valueOf(_imu.getPitch()));
        telemetry.addLine(String.valueOf(_imu.getRoll()));
    }

    @Override
    public void loop() {}
}