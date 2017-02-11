package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Bots of Prey on 2/11/2017.
 */

public class RpsCounter {

    private final DcMotor motor;
    private final double ticksPerRevolution;
    private final long sampleDuration;
    private long lastSampleTimeInMillis;
    private long lastPositionInTicks;
    private long currSampleTimeInMillis;
    private long currPositionInTicks;

    public RpsCounter(DcMotor motor, int ticksPerRevolution, long sampleDuration) {
        this.motor = motor;
        this.ticksPerRevolution = ticksPerRevolution;
        this.sampleDuration = sampleDuration;
        this.lastSampleTimeInMillis = System.currentTimeMillis() - sampleDuration;
        this.lastPositionInTicks = motor.getCurrentPosition();
        this.currSampleTimeInMillis = System.currentTimeMillis();
        this.currPositionInTicks = motor.getCurrentPosition();
    }

    // update last time and last encoder position if sample duration time has passed
    public void update() {
        if (System.currentTimeMillis() >= currSampleTimeInMillis + sampleDuration) {
            lastSampleTimeInMillis = currSampleTimeInMillis;
            lastPositionInTicks = currPositionInTicks;
            currSampleTimeInMillis = System.currentTimeMillis();
            currPositionInTicks = motor.getCurrentPosition();
        }
    }

    // calculate RPS from current time/encoder position and last time/encoder position
    public double getRps() {
        double deltaTimeInSec = (currSampleTimeInMillis - lastSampleTimeInMillis) / 1000.0;
        double deltaPostion = Math.abs(currPositionInTicks - lastPositionInTicks);
        double ticksPerSecond = deltaPostion / deltaTimeInSec;
        double rps = ticksPerSecond / ticksPerRevolution;
        return rps;
    }

}
