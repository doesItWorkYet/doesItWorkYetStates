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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Zero servos", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class servoZeroControlled extends LinearOpMode {
    HardwareMapLucyV4 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero(this);
        double beaconPresserRightPosition = 0;
        double beaconPresserLeftPosition = 0;
        double armLeftPosition = 0;
        double armRightPosition = 0;
        double deflectorPosition = 0;
        double indexerPosition = 0;
        //Wait for start and reset the runtime count
        waitForStart();
       while(opModeIsActive()){
           if(gamepad2.dpad_up) {
               if (gamepad2.a) beaconPresserRightPosition = Math.min(180, beaconPresserRightPosition + 1);
               if (gamepad2.b) beaconPresserLeftPosition = Math.min(180, beaconPresserLeftPosition + 1);
               if (gamepad2.y) armLeftPosition = Math.min(180, armLeftPosition + 1);
               if (gamepad2.x) armRightPosition = Math.min(180, armRightPosition + 1);
               if (gamepad2.right_bumper) deflectorPosition = Math.min(180, deflectorPosition + 1);
               if (gamepad2.left_bumper) indexerPosition = Math.min(180, indexerPosition + 1);
           }
           if(gamepad2.dpad_down){
               if (gamepad2.a) beaconPresserRightPosition = Math.max(0, beaconPresserRightPosition - 1);
               if (gamepad2.b) beaconPresserLeftPosition = Math.max(0, beaconPresserLeftPosition - 1);
               if (gamepad2.y) armLeftPosition = Math.max(0, armLeftPosition - 1);
               if (gamepad2.x) armRightPosition = Math.max(0, armRightPosition - 1);
               if (gamepad2.right_bumper) deflectorPosition = Math.max(0, deflectorPosition - 1);
               if (gamepad2.left_bumper) indexerPosition = Math.max(0, indexerPosition - 1);
           }

           robot.indexer.setPosition(indexerPosition/180.0);
           robot.armletLeft.setPosition(armLeftPosition/180.0);
           robot.armletRight.setPosition(armRightPosition/180.0);
           robot.beaconPresserRight.setPosition(beaconPresserRightPosition/180.0);
           robot.beaconPresserLeft.setPosition(beaconPresserLeftPosition/180.0);
           robot.flyWheelDeflector.setPosition(deflectorPosition/180.0);
           telemetry.addData("---Positions", "---");
           telemetry.addData("---------------", "");
           telemetry.addData("Indexer", robot.indexer.getPosition()*180);
           telemetry.addData("Deflector", robot.flyWheelDeflector.getPosition()*180);
           telemetry.addData("Beacon Presser Left", robot.beaconPresserLeft.getPosition()*180);
           telemetry.addData("Beacon Presser Right", robot.beaconPresserRight.getPosition()*180);
           telemetry.addData("Left Arm", robot.armletLeft.getPosition()*180);
           telemetry.addData("Right Arm", robot.armletRight.getPosition()*180);
           telemetry.addData("---------------", "");
           telemetry.update();

           idle();
       }

    }

}

