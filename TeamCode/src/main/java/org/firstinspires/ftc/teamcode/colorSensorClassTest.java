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
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="rgbSensorTest", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class colorSensorClassTest extends LinearOpMode {
    final double WHITE_FUDGE_FACTOR = .2;
    final int rgbSensorLED = 5;
    final boolean rgbSensorLedState = true;
    ColorSensor SensorRGB = null;
    DeviceInterfaceModule cdim = null;
    private ElapsedTime runtime = new ElapsedTime();
    rgbSensor colorSensor1;
    double[] baseLineColorAverage = {0.0,0};
    final int NUM_TIMES_TO_CHECK = 10;
    @Override
    public void runOpMode() throws InterruptedException {
    cdim = hardwareMap.deviceInterfaceModule.get("dim");
        SensorRGB = hardwareMap.colorSensor.get("colorSensor");
        colorSensor1 = new rgbSensor(SensorRGB,cdim,rgbSensorLED,rgbSensorLedState);

        waitForStart();
        //get baseline state
        baseLineColorAverage = getBaseLineColorState();
        while(opModeIsActive()){
            double[] rgbValues = colorSensor1.getRGBColor();
            telemetry.addData("Red: " ,new Double(rgbValues[0]).toString());
            telemetry.addData("Green: " ,new Double(rgbValues[1]).toString());
            telemetry.addData("Blue: " ,new Double(rgbValues[2]).toString());
            boolean isWhite = checkIfWhite(rgbValues);
            telemetry.addData("White: ", isWhite);
            telemetry.update();
            idle();
        }

    }

    private boolean checkIfWhite(double[] rgbValuesToCheck) {
        if (rgbValuesToCheck[0] > baseLineColorAverage[0] * (1 + WHITE_FUDGE_FACTOR)) {
            if (rgbValuesToCheck[1] > baseLineColorAverage[1] *(1 + WHITE_FUDGE_FACTOR)) {
                if (rgbValuesToCheck[2] > baseLineColorAverage[2] * (1 + WHITE_FUDGE_FACTOR)) {
                    return true;
                }
            }
        }
        return false;
    }

    private double[] getBaseLineColorState() {
       double[] toReturn = {0,0,0};
        double[] rgbValues = colorSensor1.getRGBColor();
        toReturn = rgbValues;
        for(int i = 0; i < NUM_TIMES_TO_CHECK; i ++){
            rgbValues = colorSensor1.getRGBColor();
            toReturn[0] = (toReturn[0] + rgbValues[0])/2.0;
            toReturn[1] = (toReturn[1] + rgbValues[1])/2.0;
            toReturn[2] = (toReturn[2] + rgbValues[2])/2.0;
        }
        return toReturn;
    }
}

