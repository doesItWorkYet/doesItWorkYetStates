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

@Autonomous(name="Turn to white", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class turnToWhite extends LinearOpMode {
    HardwareMapLucyV4 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero(this);

        boolean safety = robot.leftBeaconPresserSensor.isPressed() || robot.rightBeaconPresserSensor.isPressed();
        //Wait for start and reset the runtime count
        waitForStart();
        long timeToStop= System.currentTimeMillis() + 30*1000;
        robot.deployBeaconPressers();
       //while(opModeIsActive()){
        robot.driveDistance(1, .25, this);
        robot.oneWheelTurn(robot.RIGHT_MOTOR, -45, .15, this);
        robot.beginSynchronousDriving(.65, .65);
        while(robot.groundColorSensor.getBrightness()<robot.BRIGHTNESS_WHITE_THRESHOLD && !safety && opModeIsActive()) {
            safety = robot.leftBeaconPresserSensor.isPressed() || robot.rightBeaconPresserSensor.isPressed();
            idle();

        }
        if(!safety && opModeIsActive()) {
            robot.endSynchronousDriving(this);
            robot.oneWheelTurn(robot.LEFT_MOTOR, -45, .15, this);
            robot.driveStraightUntilWall(.15, robot.OPTICAL_SENSOR_THRESHOLD,timeToStop, this);

        }

       //}

    }

}

