package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="TestCarousel", group="basic")
public class TestCarousel extends AutonomousControl
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.carousel);

        rob.runCarouselsTimeSpeed(-0.9, 5000);
    }
}