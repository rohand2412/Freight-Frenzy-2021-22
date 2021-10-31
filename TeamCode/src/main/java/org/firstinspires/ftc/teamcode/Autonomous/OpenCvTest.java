
/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//hi

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="OpenCV test", group = "basic")
public class OpenCvTest extends AutonomousControl
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
        // landscape orientation, though.

        rob.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                rob.webcam.startStreaming(320,240);
            }
        });

        double currTime = runtime.milliseconds();

        waitForStart();
        double x = 0.0;

            //rob.driveTrainEncoderMovement(0.5, 10, 4, 0, Goal.movements.forward);
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("Value", pipeline.value);
        telemetry.addData("US value", "%.2f cm", rob.Back.getDistance(DistanceUnit.CM));
        telemetry.addData("comp", "%.2f cm", x);
        telemetry.update();



        /*while((x<91) || (x > 1000)){
                rob.driveTrainEncoderMovement(0.6, 5, 10, 0, Goal.movements.forward);
                sleep(50);
                x = rob.Back.getDistance(DistanceUnit.CM);
               telemetry.update();
               sleep(50);
        }
*/

        rob.driveTrainEncoderMovement(0.8, 36, 1000, 0, Goal.movements.forward);

        int r = (int) rob.Back.getDistance(DistanceUnit.CM);

        while(r!=91){
            if(r>91 && !outlier()){
                rob.driveTrainEncoderMovement(0.2, 0.4, 10, 0, Goal.movements.backward);
            }
            else if(!outlier()){
                rob.driveTrainEncoderMovement(0.2, 0.4, 10, 0, Goal.movements.forward);
            }
            r = (int) rob.Back.getDistance(DistanceUnit.CM);
        }

            while(opModeIsActive()){
               telemetry.update();
            }

        /*    telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            telemetry.update();
            if (pipeline.value == 4) {

            } else if (pipeline.value == 1) {

            } else {

            }
*/

    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
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
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(150,148);

        static final int REGION_WIDTH = 150;
        static final int REGION_HEIGHT = 150;

        final int FOUR_RING_THRESHOLD = 147;
        final int ONE_RING_THRESHOLD = 135;
        final int ZERO_RING_THRESHOLD = 120;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */


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
            }else if (avg1 < 140){
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