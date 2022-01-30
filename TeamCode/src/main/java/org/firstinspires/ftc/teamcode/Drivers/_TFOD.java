package org.firstinspires.ftc.teamcode.Drivers;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Control.Robot;

import java.util.List;

public class _TFOD {

    private final double _INTERVAL_MS;

    private final TFObjectDetector _tfod;
    private List<Recognition> _recognitions;
    private double _lastUpdatedTime;
    private boolean _willUpdate;

    public _TFOD(VuforiaLocalizer vuforia, float confidenceThreshold, boolean isTF2, int inputSize, double zoom, double aspectRatio, String model, String[] labels, double intervalMS, boolean willUpdate) {
        _INTERVAL_MS = intervalMS;
        _willUpdate = willUpdate;

        int tfodMonitorViewId = Robot.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", Robot.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = confidenceThreshold;
        tfodParameters.isModelTensorFlow2 = isTF2;
        tfodParameters.inputSize = inputSize;
        _tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        _tfod.loadModelFromAsset(model, labels);

        //Activate TensorFlow Object Detection before we wait for the start command.
        //Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
        if (_tfod != null) {
            _tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            _tfod.setZoom(zoom, aspectRatio);
        }
    }

    public _TFOD(VuforiaLocalizer vuforia, float confidenceThreshold, boolean isTF2, int inputSize, double zoom, double aspectRatio, String model, String[] labels, boolean willUpdate) {
        this(vuforia, confidenceThreshold, isTF2, inputSize, zoom, aspectRatio, model, labels, 100, willUpdate);
    }

    public void willUpdate(boolean willUpdate) {
        _willUpdate = willUpdate;
        if (_willUpdate) {
            _tfod.getUpdatedRecognitions();
        }
    }

    public void update() {
        if (_willUpdate && (Robot.runtime.milliseconds() >= _lastUpdatedTime + _INTERVAL_MS)) {
            _recognitions = _tfod.getUpdatedRecognitions();

            _lastUpdatedTime = Robot.runtime.milliseconds();
        }
    }

    public Recognition getRecognitionValidLabel(ValidRecognition validRecognition, String label) {
        for (int i = 0; i < _recognitions.size(); i++) {
            if (_recognitions.get(i) != null) {
                if (validRecognition.check(_recognitions.get(i)) && _recognitions.get(i).getLabel().equals(label)) {
                    return _recognitions.get(i);
                }
            }
        }
        return null;
    }

    public int countValidLabel(ValidRecognition validRecognition, String label) {
        int count = 0;
        for (Recognition recognition : _recognitions) {
            if (recognition != null) {
                if (validRecognition.check(recognition) && recognition.getLabel().equals(label)) {
                    count++;
                }
            }
        }
        return count;
    }

    public List<Recognition> getRecognitions() {
        return _recognitions;
    }

    public interface ValidRecognition {
        boolean check(Recognition recognition);
    }
}