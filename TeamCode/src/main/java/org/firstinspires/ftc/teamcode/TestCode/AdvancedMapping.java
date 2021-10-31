package org.firstinspires.ftc.teamcode.TestCode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name = "AdvancedMapping", group = "basic")

public class AdvancedMapping extends AutonomousControl {

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.autonomous);
        telemetry.addLine("Start!");
        telemetry.update();
        if (opModeIsActive()) {

            double distX = rob.Right.getDistance(DistanceUnit.INCH) + (13.229/2);
            double distY = rob.Back.getDistance(DistanceUnit.INCH) + (15.118 / 2);

//            int[][] squares = {{distX,distY},{0,24},{0,48},{0,72},{0,96},{0,120},{0,144},
//                    {24,0},{24,24},{24,48},{24,72},{24,96},{24,120},{24,144},
//                    {48,0},{48,24},{48,48},{48,72},{48,96},{48,120},{48,144},
//                    {72,0},{72,24},{72,48},{72,72},{72,96},{72,120},{72,144},
//                    {96,0},{96,24},{96,48},{96,72},{96,96},{96,120},{96,144}};

            double[][] squares = new double[12][2];
            // [i][0] is right, [i][1] is back
            int p = 0;
            for (int i=0; i<3; i++) {
                for (int j=0; j<4; j++) {
                    squares[p][0] = 12 + 24*j;
                    squares[p][1] = 12 + 24*i;
                    p++;
                }
            }

            for (int i=0; i<12; i++ ) {
                System.out.println(squares[i][0] + " " + squares[i][1]);
            }

            // find closest
            double min=dist(distX, distY, squares[0][0], squares[0][1]);
            int closeindex = 0;
            for (int i=1; i<12; i++) {
                double curdist = dist(distX, distY, squares[i][0], squares[i][1]);
                if (curdist < min) {
                    min = curdist;
                    closeindex = i;
                }
            }

            rob.driveTrainEncoderMovement(0.2, distX - squares[closeindex][0], 20, 0, Goal.movements.right);
            rob.driveTrainEncoderMovement(0.2, distY - squares[closeindex][1], 20, 0, Goal.movements.backward);

            // go to the location

           /* int[][] squares = { {0,0},{0,24},{0,48},{0,72},{0,96},{0,120},{0,144},
                    {24,0},{24,24},{24,48},{24,72},{24,96},{24,120},{24,144},
                    {48,0},{48,24},{48,48},{48,72},{48,96},{48,120},{48,144},
                    {72,0},{72,24},{72,48},{72,72},{72,96},{72,120},{72,144},
                    {96,0},{96,24},{96,48},{96,72},{96,96},{96,120},{96,144}}; //list of x, and y coordinates of each square.
            int x = (int)(rob.Right.getDistance(DistanceUnit.CM)/2.54);
            int y = (int)(rob.Back.getDistance(DistanceUnit.CM)/2.54);
            //where the robot is right now.
            int i = 0; // we init the value here so that we can use this variable out of the for loop.
            for (; i < squares.length; i++){
                if (squares[i][0]>x){ // finding out which square the robot is in according to the array
                    break;
                }
            }
            i--;
            int j = 0; // we init the value here so that we can use this variable out of the for loop.
            for (; j < squares.length; j++){
                if (squares[j][0]>y){ // finding out which square the robot is in according to the array
                    break;
                }
            }
            j--;
            int x1 = squares[i][0]; // the x coordinate the robot is in
            int y1 = squares[j][1]; // the y coordinate the robot is in
            */
//            double distX = rob.Right.getDistance(DistanceUnit.INCH) + (13.229/2);
//            double distY = rob.Back.getDistance(DistanceUnit.INCH) + (15.118 / 2);

            /*
            (0,0)     (0, 12)    (0, 24)
            (12, 0)   (12, 12)   (12, 24)
            (24, 0)   (24, 12)   (24, 24)

             */

//            rob.stopDrivetrain();
//            telemetry.addData("right", distX);
//            telemetry.addData("right mod", distX%24);
//            telemetry.addData("back", distY);
//            telemetry.update();
//            sleep(5000);

//            rob.driveTrainEncoderMovement(0.2,distY%12 ,20,0,Goal.movements.forward);
//
//            if (distX%24 > 12) {
//                rob.driveTrainEncoderMovement(0.2, distX%24 - 12, 20, 0, Goal.movements.left);
//            }
//            else {
//                rob.driveTrainEncoderMovement(0.2, 12 - distX%24, 20, 0, Goal.movements.right);
//            }

        /*
            rob.stopDrivetrain();
            telemetry.addData("right", distX);
            telemetry.addData("right mod", distX%12);
            telemetry.addData("back", distY);
            telemetry.update();
            sleep(5000);
            double close = (distX - (distX%12));
            double closeY = (distY - (distY%12));
            if(close % 24 == 0){
                close = distX + (12 - (distX%12));
            }
            telemetry.addData("DistX: ", distX);
            telemetry.addData("distY: ", distY);
            telemetry.addData("close: ", close);
            telemetry.addData("closeY: ", closeY);
            telemetry.update();
            sleep(5000);
            if(close > distX){
                rob.driveTrainEncoderMovement(0.2, close - distX - 2, 20, 0, Goal.movements.left);
            }
            else{
                rob.driveTrainEncoderMovement(0.2, distX - close - 2, 20, 0, Goal.movements.right);
            }
            if(closeY > distY){
                rob.driveTrainEncoderMovement(0.2, closeY - distY - 2, 20, 0, Goal.movements.forward);
            }
            else{
                rob.driveTrainEncoderMovement(0.2, distY - closeY - 2, 20, 0, Goal.movements.backward);

            }
            telemetry.update();
        */
/*
            double dist = 0;
//            dist = rob.Back.getDistance(DistanceUnit.CM) / 2.54;
            dist = rob.Back.getDistance(DistanceUnit.INCH);
            dist += (15.118 / 2);
            int squareBack = (int) (dist) % 24;
            int squareRight = (int) (((rob.Right.getDistance(DistanceUnit.CM) / 2.54)) - (13.229 / 2)) % 24;
            int originalBack = squareBack;
            int originalRight = squareRight;
            if (originalBack < 12) {
//                do{
//                    rob.driveTrainMovement(0.2, Goal.movements.forward);
//                    dist = rob.Back.getDistance(DistanceUnit.INCH);
//                    dist += (15.118 / 2);
//                    squareBack = (int) (dist) % 24;
//                    telemetry.addData("squareBack: ", squareBack);
//                    telemetry.update();
//                }
//                while (dist > 1000 || dist < 12 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                while (squareBack < 12) {
                    rob.driveTrainMovement(0.2, Goal.movements.forward);
                    dist = rob.Back.getDistance(DistanceUnit.INCH);
                    dist += (15.118 / 2);
                    squareBack = (int) (dist) % 24;
                    telemetry.addData("squareBack: ", squareBack);
                    telemetry.update();
                }
            }
            if (originalBack > 12) {
//                do{
//                    rob.driveTrainMovement(0.2, Goal.movements.backward);
//                    dist = rob.Back.getDistance(DistanceUnit.INCH);
//                    dist += (15.118 / 2);
//                    squareBack = (int) (dist) % 24;
//                    telemetry.addData("squareBack: ", squareBack);
//                    telemetry.update();
//                }
//                while (dist > 1000 || dist > 12 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
                while (squareBack > 12) {
                    rob.driveTrainMovement(0.2, Goal.movements.backward);
                    dist = rob.Back.getDistance(DistanceUnit.INCH);
                    dist += (15.118 / 2);
                    squareBack = (int) (dist) % 24;
                    telemetry.addData("squareBack: ", squareBack);
                    telemetry.update();
                }
            }
*/

/*
           if(originalRight<12){
               while(squareRight<12){
                   rob.driveTrainMovement(0.6, Goal.movements.left);
                   squareBack = (int)(((rob.Back.getDistance(DistanceUnit.CM)/2.54)/24)+(15.118/2))%24;
               }
           }
            if(originalRight>12){
                while(squareRight>12){
                    rob.driveTrainMovement(0.6, Goal.movements.right);
                    squareBack = (int)(((rob.Back.getDistance(DistanceUnit.CM)/2.54)/24)+(15.118/2))%24;
                }
            }
*/
        }
    }

    public double dist(double x1, double y1, double x2, double y2) {
        return (double)(x1 - x2)*(x1 - x2) + (double)(y1 - y2) * (y1 - y2);
    }
}