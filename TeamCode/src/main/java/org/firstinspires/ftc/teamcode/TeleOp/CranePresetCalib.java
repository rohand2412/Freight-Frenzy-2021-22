package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Goal;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.teamcode.Control.Constants.BUCKET_PRELOAD_DEGREES;
import static org.firstinspires.ftc.teamcode.Control.Constants.DEAD_ZONE_SIZE;

@TeleOp(name="CranePresetCalib", group = "basic")
public class CranePresetCalib extends TeleOpControl
{
    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Goal.setupType.crane, Goal.setupType.bucket);

        double liftDegrees = 0;
        double bucketDegrees = BUCKET_PRELOAD_DEGREES;

        while (opModeIsActive()) {
            standardGamepadData();

            if (rb) { rob.liftCrane(0.2, liftDegrees + 5); liftDegrees += 5; }
            else if (lb) { rob.liftCrane(0.2, liftDegrees - 5); liftDegrees -= 5; }

            if (rt > DEAD_ZONE_SIZE) { rob.bucketSetDegree(bucketDegrees + 0.5); bucketDegrees += 0.5; }
            else if (lt > DEAD_ZONE_SIZE) { rob.bucketSetDegree(bucketDegrees - 0.5); bucketDegrees -= 0.5; }

            telemetry.addLine("Lift: " + liftDegrees);
            telemetry.addLine("Bucket: " + bucketDegrees);
            telemetry.update();
        }
    }
}