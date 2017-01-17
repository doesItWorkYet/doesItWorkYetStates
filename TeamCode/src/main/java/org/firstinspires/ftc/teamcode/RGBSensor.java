package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDeviceImpl;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;

import java.lang.reflect.Field;

/**
 * Created by root on 12/19/16.
 */
public class RGBSensor {
    private boolean ledOn;
    private DeviceInterfaceModule dim;
    private HardwareMap hwMap;
    private LED power;
    private int ledPin;
    private int powerPin;
    private int chanel;
    private ColorSensor colorSensor;
    private float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;
    private String name;

    RGBSensor(String name, HardwareMap hw, DeviceInterfaceModule dim, int ledPin, boolean ledValue){
        this.hwMap = hw;
        this.colorSensor = this.hwMap.colorSensor.get(name);
        this.ledPin = ledPin;
        this.ledOn = ledValue;
        this.dim = dim;
        this.name = name;
        this.powerPin = powerPin;
        //this.dim.setDigitalChannelMode(this.powerPin, DigitalChannelController.Mode.OUTPUT);
        //this.dim.setDigitalChannelState(this.powerPin, false);
        this.dim.setDigitalChannelMode(this.ledPin, DigitalChannelController.Mode.OUTPUT);
        if(ledValue = true) turnLedOn();
        else turnLedOff();

    }

    public void waitForInitialization(){

        this.turnSensorOn();
        //delay(800);
        //this.dim.resetDeviceConfigurationForOpMode();
        //this.colorSensor.resetDeviceConfigurationForOpMode();
        //delay(700);
        //this.colorSensor = hwMap.colorSensor.get(this.name);
        delay(800);
        //double[] rgb = getRGBColor();
        //while(rgb[0] == 0) rgb = getRGBColor();
    }

    private I2cController getI2cController() {
        I2cController controller = null;
        Class<I2cControllerPortDeviceImpl> i2cControllerClass = I2cControllerPortDeviceImpl.class;
        try {
            Field controllerField = i2cControllerClass.getDeclaredField("controller");
            controllerField.setAccessible(true);
            controller = (I2cController) controllerField.get(this.colorSensor);
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        return controller;
    }


    public void delay(long time){
        //long curTime = System.currentTimeMillis();
        //while(System.currentTimeMillis() < time + curTime);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            //intentionally empty
        }

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
        //this.dim.setDigitalChannelMode(this.powerPin, DigitalChannelController.Mode.OUTPUT);
        //I2cController controller = getI2cController();
       // controller.i
          //      onModuleStateChange(null, RobotArmingStateNotifier.ARMINGSTATE.DISARMED);
        ((AdafruitI2cColorSensor) this.colorSensor).onModuleStateChange(null, RobotArmingStateNotifier.ARMINGSTATE.DISARMED);
        this.dim.setDigitalChannelState(this.powerPin, false);
    }
    private void turnSensorOn(){
        //this.dim.setDigitalChannelMode(this.powerPin, DigitalChannelController.Mode.OUTPUT);
        this.dim.setDigitalChannelState(this.powerPin, true);
        delay(300);

        ((AdafruitI2cColorSensor) this.colorSensor).onModuleStateChange(null, RobotArmingStateNotifier.ARMINGSTATE.ARMED);
        //this.dim.setLED(this.powerPin, false);
    }


    public void turnLedOn(){
        this.dim.setDigitalChannelState(this.ledPin, false);
    }
    public void turnLedOff(){
        this.dim.setDigitalChannelState(this.ledPin, true);
    }
}
