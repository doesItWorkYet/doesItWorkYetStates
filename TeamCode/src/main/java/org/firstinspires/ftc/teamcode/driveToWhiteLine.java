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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Drive to white", group="Testing")  // @Autonomous(...) is the other common choice
@Disabled
public class driveToWhiteLine extends LinearOpMode {
    HardwareMapLucyV4 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero(this);
        //Wait for start and reset the runtime count
        waitForStart();
        //begin
        //loop
        //end
        telemetry.addData("Start robot", "");
        telemetry.update();
        robot.deployBeaconPressers();
        telemetry.addData("Deploy beacon pressers and wait", "");
        telemetry.update();
        robot.delay(100);
        while(robot.leftBeaconPresserSensor.isPressed() || robot.rightBeaconPresserSensor.isPressed());
        robot.delay(400);
        robot.beginSynchronousDriving(.65);
        telemetry.addData("Start Sync Driving", "");
        telemetry.update();
       while(robot.groundColorSensor.getBrightness() < robot.BRIGHTNESS_WHITE_THREASHOLD && !robot.rightBeaconPresserSensor.isPressed() && !robot.leftBeaconPresserSensor.isPressed() && opModeIsActive()){
           telemetry.addData("L: ", robot.groundColorSensor.getBrightness());
           telemetry.update();
           idle();
       }
        robot.endSynchronousDriving();
        telemetry.addData("Stop Sync Driving", "");
        telemetry.update();
        robot.oneWheelTurn(robot.LEFT_MOTOR, 90, .15);
        telemetry.addData("Robot turns 90 degreees", "");
        telemetry.update();


    }

}

