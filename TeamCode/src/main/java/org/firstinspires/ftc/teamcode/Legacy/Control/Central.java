package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Central extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();

    public Goal rob;

    public Goal.movements[] allMovements = {Goal.movements.forward, Goal.movements.backward, Goal.movements.right, Goal.movements.left, Goal.movements.tr, Goal.movements.bl, Goal.movements.tl, Goal.movements.br, Goal.movements.cw, Goal.movements.ccw};

    public void setRob(Goal rob) {
        this.rob = rob;
    }

    public void setup(ElapsedTime rtime, Goal.setupType... setupModes) throws InterruptedException {
        setup(rtime, true, setupModes);
    }

    public void setup(ElapsedTime rtime, boolean waitAndStart, Goal.setupType... setup) throws InterruptedException {
        this.setRob(new Goal(hardwareMap, runtime, this, setup));
        setRuntime(rtime);
        /*if (rob.vuforiaMode){
            //    rob.vuforia.targetsRoverRuckus.activate();
        }
        if (rob.tensorflowMode){
            rob.vuforia.tfod.activate();
        }*/

        if (waitAndStart) { waitAndStart(); }
    }

    public void waitAndStart() throws InterruptedException {
        this.waitForStart();
        this.runtime.reset();
    }

    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }


}
