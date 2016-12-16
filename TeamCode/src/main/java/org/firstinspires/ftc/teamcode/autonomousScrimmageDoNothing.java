/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

import com.qualcomm.ftccommon.Device;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

@Autonomous(name="autonomousScrimmageDoNothing", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class autonomousScrimmageDoNothing extends LinearOpMode implements SensorEventListener{
    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];
    SensorManager sensorService;
    Sensor mag;
    Sensor accel;
    String toPost;
    /* Declare OpMode members. */
    ColorSensor SensorRGB = null;
    DeviceInterfaceModule cdim = null;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sensorService = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        accel = sensorService.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mag = sensorService.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        SensorRGB = hardwareMap.colorSensor.get("color");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
       waitForStart();
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        double orientation[] = getOrientation();
        //z is in orientation[0]!!!!!!

        /*
        while (opModeIsActive()) {
            Color.RGBToHSV((SensorRGB.red() * 255) / 800, (SensorRGB.green() * 255) / 800, (SensorRGB.blue() * 255) / 800, hsvValues);
            telemetry.addData("Clear", SensorRGB.alpha());
            telemetry.addData("Red ", SensorRGB.red());
            telemetry.addData("Green", SensorRGB.green());
            telemetry.addData("Blue ", SensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

            idle();
        }
        */

        runtime.reset();

        }

    public double[] getOrientation(){
        //returns in an array [z,x,y]


        // Rotation matrix based on current readings from accelerometer and magnetometer.
        // Rotation matrix based on current readings from accelerometer and magnetometer.
        final float[] rotationMatrix = new float[9];
        float I[] = new float[9];
        sensorService.getRotationMatrix(rotationMatrix, I, accelerometerReading, magnetometerReading);
        // Express the updated rotation matrix as three orientation angles.
        final float[] orientationAngles = new float[3];
        sensorService.getOrientation(rotationMatrix, orientationAngles);
        final double[] myOrientaion = {(int) Math.round(Math.toDegrees(orientationAngles[0])),orientationAngles[1]*180.0/Math.PI,orientationAngles[2]*180.0/Math.PI};
        return myOrientaion;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
            //Log.d("Accel", "1");
            System.arraycopy(event.values, 0, accelerometerReading,0, accelerometerReading.length);
        }
        if(event.sensor.getType()==Sensor.TYPE_MAGNETIC_FIELD){
            System.arraycopy(event.values, 0, magnetometerReading, 0, magnetometerReading.length);
            //Log.d("MAG", "1");
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}

