package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by root on 2/11/17.
 */
public class motorProprtionalController {
    HardwareMapLucyV4 robot;
    RpsCounter rpsCounter;
    DcMotor motor;
    long ticksPerRev;
    double maxRps;
    double desiredRps;
    double tuningP;


    public motorProprtionalController(DcMotor motorObj,long ticksPerRevolution,double maxRps, double p){
        this.motor = motorObj;
        this.ticksPerRev = ticksPerRevolution;
        this.maxRps = maxRps;
        this.tuningP = p;
        rpsCounter = new RpsCounter(motor,(int)ticksPerRev,200);
    }

    public void setPower(double power){
        if(power == 0) desiredRps = 0;
        this.desiredRps = power*maxRps;
    }

    public void update(){
        if(desiredRps != 0) {
            rpsCounter.update();
            double currentRps = rpsCounter.getRps();
            double error = this.desiredRps - currentRps;
            double newPower = desiredRps/maxRps + error * tuningP;
            if (newPower < 0) newPower = 0;
            if (newPower > 1) newPower = 1;
            motor.setPower(newPower);
        }
        else motor.setPower(0);
    }
}
