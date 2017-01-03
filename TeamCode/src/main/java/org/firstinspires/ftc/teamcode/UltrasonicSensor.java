package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by root on 1/3/17.
 */
public class UltrasonicSensor {
    private DeviceInterfaceModule dim;
    private int readPin, trigPin;
    UltrasonicSensor(int readPin, int trigPin, DeviceInterfaceModule dim){
        this.dim = dim;
        this.readPin = readPin;
        this.trigPin = trigPin;
        this.dim.setDigitalChannelMode(this.readPin, DigitalChannelController.Mode.INPUT);
    }

    public double getDistance(){
        fire();
        long time = pulseIn(this.trigPin);
        double dist = time /2.0/34.0;
        return dist;
    }

    private long pulseIn(int pin){
        long curTime = System.currentTimeMillis();
        while(!this.dim.getDigitalChannelState(this.readPin));
        return System.currentTimeMillis() - curTime;
    }
    private void fire(){
        this.dim.setDigitalChannelState(this.trigPin, true);
        sleepMillis(3);
        this.dim.setDigitalChannelState(this.trigPin, false);
    }

    private void sleepMillis(long time){
        long curTime = System.currentTimeMillis();
        while(curTime + time < System.currentTimeMillis());
    }
}
