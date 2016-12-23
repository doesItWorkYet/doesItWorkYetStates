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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RunToWhite", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class goForwardUntilWhite extends LinearOpMode {
    HardwareMapLucyV4 robot;
    private MotorController driveLeftController, driveRightController;
    double[] baseLineColorAverage = {0,0,0};
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero();
        //Wait for start and reset the runtime count
        baseLineColorAverage = getBaseLineColorState();
        robot.colorSensor.turnLedOn();
        waitForStart();
        boolean runTimes = false;
       while(opModeIsActive()){
           double[] newColor = getColor();
           telemetry.addData("Color Sensor", "Color: "+robot.colorSensor.getRGBColor()[0] + " " + robot.colorSensor.getRGBColor()[1] + " " + robot.colorSensor.getRGBColor()[2]);
           telemetry.addData("Left Motor", "Power: "+robot.leftMotor.getPower());
           telemetry.addData("Right Motor", "Power: "+robot.rightMotor.getPower());
           telemetry.addData("Color Sensor", "Has Passed White: "+runTimes);
           telemetry.addData("baseline: ", baseLineColorAverage[0] + " " + baseLineColorAverage[1] + " " + baseLineColorAverage[2]);
           telemetry.addData("White check: " , checkIfWhite(newColor));
           telemetry.update();

           if(!checkIfWhite(newColor) && !runTimes) {

               robot.leftMotorController.accelationRuntime(false);
               robot.rightMotorController.accelationRuntime(false);
           }
           else {
               runTimes = true;
               robot.leftMotorController.stationaryRuntime();
               robot.rightMotorController.stationaryRuntime();
           }
           idle();
       }

    }

    public double[] getColor(){
        double[] colorRGB = robot.colorSensor.getRGBColor();
        return colorRGB;
    }
    public boolean checkIfWhite(double[] rgbValuesToCheck) {
        if (rgbValuesToCheck[0] > baseLineColorAverage[0] * (1 + robot.WHITE_FUDGE_FACTOR)) {
            if (rgbValuesToCheck[1] > baseLineColorAverage[1] *(1 + robot.WHITE_FUDGE_FACTOR)) {
                if (rgbValuesToCheck[2] > baseLineColorAverage[2] * (1 + robot.WHITE_FUDGE_FACTOR)) {
                    return true;
                }
            }
        }
        return false;
    }

    public double[] getBaseLineColorState() {
        double[] toReturn = {0,0,0};
        double[] rgbValues = getColor();
        toReturn = rgbValues;
        for(int i = 0; i < robot.COLOR_SENSOR_NUM_TIMES_CHECK_BACKGROUND_COLOR; i ++){
            rgbValues = getColor();
            toReturn[0] = (toReturn[0] + rgbValues[0])/2.0;
            toReturn[1] = (toReturn[1] + rgbValues[1])/2.0;
            toReturn[2] = (toReturn[2] + rgbValues[2])/2.0;
        }
        return toReturn;
    }
}

