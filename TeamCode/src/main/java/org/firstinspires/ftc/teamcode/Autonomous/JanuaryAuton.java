
/*


package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="January Auton", group = "basic")
public class JanuaryAuton extends AutonomousControl
{
    SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(runtime, Goal.setupType.openCV, Goal.setupType.autonomous);
        telemetry.addLine("Start!");
        telemetry.update();

        pipeline = new SkystoneDeterminationPipeline();
        rob.webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.h

        rob.webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        rob.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                rob.webcam.startStreaming(320,240);
            }
        });

        double currTime = runtime.milliseconds();
//hi
        waitForStart();

        if (opModeIsActive())
        {
            double distanceBack = rob.Back.getDistance(DistanceUnit.CM);
            double distanceFront = rob.Front.getDistance(DistanceUnit.CM);
            double distanceLeft = rob.Left.getDistance(DistanceUnit.CM);
            double distanceRight = rob.Right.getDistance(DistanceUnit.CM);


            do{
                rob.driveTrainMovement(0.5, Goal.movements.forward);
                distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                telemetry.addData("cm Back", "%.2f cm", distanceBack);
                telemetry.update();

            }
            while(distanceBack >1000 || distanceBack < 34 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

            do{
                rob.driveTrainMovement(0.5, Goal.movements.left);
                distanceRight = rob.Right.getDistance(DistanceUnit.INCH);
                telemetry.addData("cm Back", "%.2f cm", distanceRight);
                telemetry.update();

            }
            while(distanceRight >1000 || distanceRight < 6 || Double.compare(distanceRight, Double.NaN) == 0 && opModeIsActive());

            rob.stopDrivetrain();

            sleep(100);

            while(opModeIsActive() && runtime.milliseconds() < 3000) {
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addData("Position", pipeline.position);
                telemetry.addData("Value", pipeline.value);
                telemetry.update();

                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
            }
            if (pipeline.value == 4){

                do{
                    rob.driveTrainMovement(0.6, Goal.movements.forward);

                    distanceFront = rob.Front.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceFront >1000 || distanceFront < 24 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                dropgoal();

                rob.driveTrainEncoderMovement(1, 45, 10, 0, Goal.movements.backward);

                do{
                    rob.driveTrainMovement(1, Goal.movements.backward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack > 21 || distanceFront < 75 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                rob.driveTrainEncoderMovement(.5, 22.5, 5, 0, Goal.movements.ccw);

                do{
                    rob.driveTrainMovement(1, Goal.movements.forward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack < 20 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                //pickupgoal();
                rob.claw.setPower(-0.4);
                sleep(200);
                rob.claw.setPower(0);
                sleep(100);

                do{
                    rob.driveTrainMovement(.5, Goal.movements.forward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack < 24 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());


                rob.stopDrivetrain();
                //taking second goal to spot

                rob.pinch.setPosition(0);
                sleep(400);

                do{
                    rob.driveTrainMovement(.5, Goal.movements.backward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack > 8 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                rob.driveTrainEncoderMovement(.5, 24, 5, 0, Goal.movements.cw);

                rob.driveTrainEncoderMovement(.5, 35, 10, 0, Goal.movements.forward);

                do{
                    rob.driveTrainMovement(.5, Goal.movements.forward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack < 53 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                dropgoal();

                do {
                    rob.driveTrainMovement(.5, Goal.movements.backward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);

                }
                while (distanceBack > 1000 || distanceBack > 49 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                do {
                    rob.driveTrainMovement(.5, Goal.movements.left);

                    distanceRight = rob.Right.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceRight);
                    telemetry.update();

                }
                while (distanceRight > 1000 || distanceRight < 28 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();

                do {
                    rob.driveTrainMovement(.5, Goal.movements.forward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);

                }
                while (distanceBack > 1000 || distanceFront < 52 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

            }


            else if(pipeline.value == 1){
/*
                //dropping off the first goal
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.forward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack < 60 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.left);
                    distanceBack = rob.Right.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm front", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack < 20 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.forward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack < 80 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
               // dropgoal();
                sleep(200);
                //go to pick up second goal
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.backward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack > 60 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                sleep(200);
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.right);
                    distanceBack = rob.Right.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm front", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack > 10 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                sleep(200);
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.backward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack > 20 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                rob.driveTrainEncoderMovement(0.6, 23, 5, 0, Goal.movements.ccw);
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.forward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack < 24 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                */

                //   pickupgoal();

                /*
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.forward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack < 24 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                pickupgoal();
                /*
                //getting the second goal
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.backward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack > 20 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                rob.driveTrainEncoderMovement(0.6, 23, 5, 0, Goal.movements.ccw);
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.forward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack < 24 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                pickupgoal();
                //taking second goal to spot
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.backward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack > 8 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                rob.driveTrainEncoderMovement(0.6, 23, 5, 0, Goal.movements.cw);
                do{
                    rob.driveTrainMovement(0.6, Goal.movements.forward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", dist);
                    telemetry.update();
                }
                while(distanceBack >1000 || distanceBack < 50 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();
                dropgoal();


            }else{
                //dropping off the first goal

                do{
                    rob.driveTrainMovement(0.6, Goal.movements.forward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack < 61 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                dropgoal();

                rob.driveTrainEncoderMovement(1, 45, 10, 0, Goal.movements.backward);

                do{
                    rob.driveTrainMovement(1, Goal.movements.backward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack > 21 || distanceFront < 75 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                rob.driveTrainEncoderMovement(.5, 22.5, 5, 0, Goal.movements.ccw);

                do{
                    rob.driveTrainMovement(1, Goal.movements.forward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack < 20 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                //pickupgoal();
                rob.claw.setPower(-0.4);
                sleep(200);
                rob.claw.setPower(0);
                sleep(100);

                do{
                    rob.driveTrainMovement(.5, Goal.movements.forward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack < 24 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());


                rob.stopDrivetrain();
                //taking second goal to spot

                rob.pinch.setPosition(0);
                sleep(400);

                do{
                    rob.driveTrainMovement(.5, Goal.movements.backward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack > 8 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                rob.driveTrainEncoderMovement(.5, 24, 5, 0, Goal.movements.cw);

                rob.driveTrainEncoderMovement(.5, 35, 10, 0, Goal.movements.forward);

                do{
                    rob.driveTrainMovement(.5, Goal.movements.forward);

                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceBack);
                    telemetry.update();

                }
                while(distanceBack >1000 || distanceBack < 53 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                rob.stopDrivetrain();

                dropgoal();

                do {
                    rob.driveTrainMovement(.5, Goal.movements.backward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);

                }
                while (distanceBack > 1000 || distanceBack > 49 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

                do {
                    rob.driveTrainMovement(.5, Goal.movements.left);

                    distanceRight = rob.Right.getDistance(DistanceUnit.INCH);
                    telemetry.addData("cm Back", "%.2f cm", distanceRight);
                    telemetry.update();

                }
                while (distanceRight > 1000 || distanceRight < 28 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());
                rob.stopDrivetrain();

                do {
                    rob.driveTrainMovement(.5, Goal.movements.forward);
                    distanceBack = rob.Back.getDistance(DistanceUnit.INCH);

                }
                while (distanceBack > 1000 || distanceFront < 52 || Double.compare(distanceBack, Double.NaN) == 0 && opModeIsActive());

            }



        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
/*
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        public int value = 0;
        /*
         * Some color constants
         */
/*
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
/*
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100,98);

        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        final int FOUR_RING_THRESHOLD = 147;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
/*
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
/*
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
                value = 4;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
                value = 1;
            }else{
                position = RingPosition.NONE;
                value = 0;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}

*/