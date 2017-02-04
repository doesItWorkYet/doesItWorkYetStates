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
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name= "Following heading to white line and turn to beacon", group="Testing")  // @Autonomous(...) is the other common choice
@Disabled
public class followHeadingToWhiteLine extends LinearOpMode {
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
        for(int i = 0; i < .6*100; i ++){
            robot.leftMotor.setPower(.2 + i/100);
            robot.rightMotor.setPower(.2 + i/100);
            idle();
        }
        robot.leftMotor.setPower(.8);
        robot.rightMotor.setPower(.8);
        telemetry.addData("Follow Heading", "");
        telemetry.update();
        robot.driveDistanceFollowingHeading(0,.8,.7,5,this);
        robot.brakeTemporarily(this);
        /*
        robot.followingHeadingToWhiteLine(0, 0.6, 0.5, this);
        telemetry.addData("Color value: ", robot.fastColorSensor.getBrightness());
        telemetry.addData("White line", "");
        telemetry.addData("stop", "");
        telemetry.update();
        robot.brakeTemporarily(this);
        robot.followingHeadingToWhiteLine(0,-0.35, -0.25, this);
        robot.brakeTemporarily(this);
        robot.turnToHeadingWithError(robot.RIGHT_MOTOR, -90, 0.2, 0.1, 3, this);
        while(opModeIsActive());
*/
    }

}

