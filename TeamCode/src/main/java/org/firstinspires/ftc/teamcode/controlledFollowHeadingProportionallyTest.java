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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Follow Heading Proportionally Controlled test", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class controlledFollowHeadingProportionallyTest extends LinearOpMode {
    HardwareMapLucyV4 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero(this);
        double heading = 0;
        double basePower = 0;
        double highPower = 0;
        //Wait for start and reset the runtime count
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                heading = 90;
                basePower = 0.3;
                highPower = 0.5;
            }
            if (gamepad1.b) {
                heading = -90;
                basePower = -0.4;
                highPower = -0.6;
            }
            if (gamepad1.x) {
                heading = 45;
                basePower = 0.5;
                highPower = 0.7;
            }
            if (gamepad1.y) {
                heading = -45;
                basePower = -0.5;
                highPower = -0.7;
            }
            if (gamepad1.dpad_up) {
                heading += 5;
                while(gamepad1.dpad_up);
            }
            if (gamepad1.dpad_down) {
                heading -= 5;
                while(gamepad1.dpad_down);
            }
            if (gamepad1.dpad_right) {
                highPower += 0.1;
                while(gamepad1.dpad_right);
            }
            if (gamepad1.dpad_left) {
                highPower -= 0.1;
                while(gamepad1.dpad_left);
            }
            if (gamepad1.left_bumper) {
                basePower += 0.1;
                while(gamepad1.left_bumper);
            }
            if (gamepad1.left_trigger > 0.1) {
                basePower -= 0.1;
                while(gamepad1.left_trigger > 0.1);
            }
            if (highPower > 1) highPower = 1;
            if (basePower > 1) basePower = 1;
            if (highPower < -1) highPower = -1;
            if (basePower < -1) basePower = -1;
            while (opModeIsActive() && gamepad1.right_trigger > 0.1) {
                while(gamepad1.right_trigger > 0.1);
                robot.driveToHeadingProportional(heading, highPower, basePower, this);
            }
            telemetry.addData("base power: ", basePower);
            telemetry.addData("high power: ", highPower);
            telemetry.addData("heading: ", heading);
            telemetry.update();
        }

    }

}

