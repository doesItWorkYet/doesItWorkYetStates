package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by root on 12/19/16.
 */
public class rgbSensor {
    private boolean ledOn;
    private DeviceInterfaceModule dim;
    private int ledPin;
    private int chanel;
    private ColorSensor colorSensor;
    private float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;

    rgbSensor(ColorSensor colorSensor, DeviceInterfaceModule dim, int ledPin, boolean ledValue){
        this.colorSensor = colorSensor;
        this.ledPin = ledPin;
        this.ledOn = ledValue;
        this.dim = dim;
        dim.setDigitalChannelMode(this.ledPin, DigitalChannelController.Mode.OUTPUT);
        if(ledValue = true) turnLedOn();
        else turnLedOff();

    }

    public double[] getRGBColor(){
        //returns rgb value 0-1024?
        double[] toReturn = {this.colorSensor.red(),this.colorSensor.green(),this.colorSensor.blue()};
        return toReturn;
    }

    public void turnLedOn(){
        this.dim.setDigitalChannelState(this.ledPin, true);
    }
    public void turnLedOff(){
        this.dim.setDigitalChannelState(this.ledPin, false);
    }
}
