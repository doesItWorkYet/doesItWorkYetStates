package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by root on 12/19/16.
 */
public class RGBSensor {
    private boolean ledOn;
    private DeviceInterfaceModule dim;
    private int ledPin;
    private int powerPin;
    private int chanel;
    private ColorSensor colorSensor;
    private float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;

    RGBSensor(ColorSensor colorSensor, DeviceInterfaceModule dim, int ledPin, boolean ledValue, int powerPin){
        this.colorSensor = colorSensor;
        this.ledPin = ledPin;
        this.ledOn = ledValue;
        this.dim = dim;
        this.powerPin = powerPin;
        dim.setDigitalChannelMode(this.powerPin, DigitalChannelController.Mode.OUTPUT);
        dim.setDigitalChannelState(this.powerPin, false);
        dim.setDigitalChannelMode(this.ledPin, DigitalChannelController.Mode.OUTPUT);
        if(ledValue = true) turnLedOn();
        else turnLedOff();

    }

    public void waitForInitialization(){
        this.turnSensorOn();
        double[] rgb = getRGBColor();
        while(rgb[0] == 0) rgb = getRGBColor();
    }



    public double[] getRGBColor(){
        //returns rgb value 0-1024?
        double[] toReturn = {this.colorSensor.red(),this.colorSensor.green(),this.colorSensor.blue()};
        return toReturn;
    }
    public double getBrightness(){
        double[] rgb = getRGBColor();
        float [] hsv = {0F,0F,0F};
        Color.RGBToHSV((int)(rgb[0]*255/800), (int)(rgb[1]*255/800),(int)(rgb[2]*255/800),hsv);
        double toReturn = (double)hsv[2]; //get the value or "Brightness"
        return toReturn;
    }

    public double[] getAverageRGBColor(int numSamples){
        double [] toReturn = {this.colorSensor.red(),this.colorSensor.green(),this.colorSensor.blue()};
        for(int i = 0; i < numSamples - 1; i ++){
            toReturn[0] = (toReturn[0] + this.colorSensor.red()) / 2.0;
            toReturn[1] = (toReturn[1] + this.colorSensor.green()) / 2.0;
            toReturn[2] = (toReturn[2] + this.colorSensor.blue()) / 2.0;
        }
        return toReturn;
    }

    public void turnSensorOff(){
        this.dim.setDigitalChannelState(this.powerPin, false);
    }
    private void turnSensorOn(){
        this.dim.setDigitalChannelState(this.powerPin, true);
    }


    public void turnLedOn(){
        this.dim.setDigitalChannelState(this.ledPin, false);
    }
    public void turnLedOff(){
        this.dim.setDigitalChannelState(this.ledPin, true);
    }
}
