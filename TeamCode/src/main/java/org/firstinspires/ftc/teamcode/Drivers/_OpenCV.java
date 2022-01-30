package org.firstinspires.ftc.teamcode.Drivers;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Control.Robot;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class _OpenCV {

    private final String _NAME;

    private final OpenCvWebcam _webcam;

    _OpenCV(String name, int width, int height, OpenCvCameraRotation rotation) {
        _NAME = name;
        int cameraMonitorViewId = Robot.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", Robot.hardwareMap.appContext.getPackageName());
        _webcam = OpenCvCameraFactory.getInstance().createWebcam(Robot.hardwareMap.get(WebcamName.class, _NAME), cameraMonitorViewId);
        _webcam.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                return input;
            }
        });
        _webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                _webcam.startStreaming(width, height, rotation);
                _webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
            }

            @Override
            public void onError(int errorCode) {
                Robot.telemetry.addLine("[ERROR] OPENCV WEBCAM ERROR: " + errorCode);
                Robot.telemetry.update();
            }
        });
    }

    _OpenCV(String name, int width, int height) {
        this(name, width, height, OpenCvCameraRotation.UPRIGHT);
    }

    public void pauseLiveView() {
        _webcam.pauseViewport();
    }

    public void resumeLiveView() {
        _webcam.resumeViewport();
    }

    public String getName() {
        return _NAME;
    }
}