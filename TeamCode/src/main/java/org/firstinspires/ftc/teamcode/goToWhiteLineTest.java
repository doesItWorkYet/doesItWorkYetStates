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

@Autonomous(name="Go to white line test", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class goToWhiteLineTest extends LinearOpMode {
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
        robot.calibrateGyro(this);
        telemetry.addData("Heading: ", robot.gyro.getIntegratedZValue());
        telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
        telemetry.update();
        robot.driveDistanceFollowingHeadingProportional(-33, 0.8, 0.5, 4, this);
        telemetry.addData("Heading: ", robot.gyro.getIntegratedZValue());
        telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
        telemetry.update();
        robot.followingHeadingToWhiteLine(-33, 0.4, 0.3, this);
        telemetry.addData("Heading: ", robot.gyro.getIntegratedZValue());
        telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
        telemetry.update();
        robot.brakeTemporarily(this);
        telemetry.addData("Heading: ", robot.gyro.getIntegratedZValue());
        telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.RIGHT_MOTOR, -60, 0.4, 0.3, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        telemetry.addData("Heading: ", robot.gyro.getIntegratedZValue());
        telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.LEFT_MOTOR, -85, 0.4, 0.3, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        while(opModeIsActive()){
            telemetry.addData("Heading: ", robot.gyro.getIntegratedZValue());
            telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
            telemetry.update();
        }
    }

}

