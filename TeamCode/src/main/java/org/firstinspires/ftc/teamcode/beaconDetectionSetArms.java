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

@Autonomous(name="beacon color tester with servo arms" , group="Testing")  // @Autonomous(...) is the other common choice

public class beaconDetectionSetArms extends LinearOpMode {
    HardwareMapLucyV4 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero();
        //Wait for start and reset the runtime count
        boolean hasMovedArms = false;
        waitForStart();
        //deploy beacon servos
        robot.deployBeaconPressers();
        robot.beaconColorSensor.waitForInitialization();
        if (robot.beaconColorSensor == null) {
            telemetry.addData("color sensor", "null");

        }
        robot.beaconColorSensor.turnLedOn();
        while (opModeIsActive()) {
            double color[] = robot.beaconColorSensor.getAverageRGBColor(10);
            telemetry.addData("R:", color[0]);
            telemetry.addData("G:", color[1]);
            telemetry.addData("B:", color[2]);
            telemetry.addData("L:", robot.beaconColorSensor.getBrightness());
            int colorOfBeacon = determineColor(color);
            if(colorOfBeacon != robot.BEACON_COLOR_UNKOWN){
                if(!hasMovedArms) {
                    if (colorOfBeacon == robot.BEACON_RED) {
                        telemetry.addData("Color of right: ", "RED" );
                        //assume color sensor is on right arm and we are red team
                        robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION);
                    } else {
                        telemetry.addData("Color of right: ", "BLUE" );
                        robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION);
                    }
                    wait(2000);
                }

            }
            else{
                telemetry.addData("Color of right: ", "Unkown");
            }
            telemetry.update();
            idle();
        }
    }


    public int determineColor(double[] rgb) {
        if (rgb[0] > robot.MIN_RED_VALUE_FOR_DETECTION) {
            if (rgb[2] > robot.MIN_BLUE_VALUE_FOR_DETECTION) {
                if (rgb[0] > rgb[2] * robot.MIN_COLOR_MULTIPLIER) {
                    return robot.BEACON_RED;
                } else if (rgb[2] > rgb[0] * robot.MIN_COLOR_MULTIPLIER) {
                    return robot.BEACON_BLUE;
                }
            }
        }
        return robot.BEACON_COLOR_UNKOWN;

    }
}
