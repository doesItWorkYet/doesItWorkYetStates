package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by root on 1/3/17.
 */
public class UltrasonicSensor {
    //TODO TEST THIS!!!!!
    private DeviceInterfaceModule dim;
    private long MAX_WAIT_TIME = 10000000;
    private int readPin, trigPin;
    UltrasonicSensor(int readPin, int trigPin, DeviceInterfaceModule dim){
        this.dim = dim;
        this.readPin = readPin;
        this.trigPin = trigPin;
        this.dim.setDigitalChannelMode(this.readPin, DigitalChannelController.Mode.INPUT);
        this.dim.setDigitalChannelMode(this.trigPin, DigitalChannelController.Mode.OUTPUT);
    }

    public double getDistance(){
        fire();
        long time = pulseIn(this.trigPin);
        double dist = time /2.0/34.0;
        return dist;
    }

    private long pulseIn(int pin){
        //wait for line to go high, then start timer, then wait until low
        long curTime = System.nanoTime();
        while(!this.dim.getDigitalChannelState(this.readPin) && System.nanoTime() - curTime < MAX_WAIT_TIME );
        curTime = System.nanoTime();
        while(this.dim.getDigitalChannelState(this.readPin) && System.nanoTime() - curTime < MAX_WAIT_TIME );
        return (System.nanoTime() - curTime)/1000; //convert to microseconds
    }
    private void fire(){
        this.dim.setDigitalChannelState(this.trigPin, true);
        sleepMicros(13);
        this.dim.setDigitalChannelState(this.trigPin, false);
    }

    private void sleepMicros(long time){
        long curTime = System.nanoTime();
        while(curTime + time*1000 < System.nanoTime());
    }
}
