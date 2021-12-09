package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name = "TestTFOD", group = "basic")
public class TestTFOD extends AutonomousControl {

    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.vuforia, Goal.setupType.tfod);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (rob.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = rob.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.addData(String.format("  confidence (%d)", i), recognition.getConfidence() + "");
                            i++;
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (rob.tfod != null) {
            rob.tfod.shutdown();
        }
    }
}
