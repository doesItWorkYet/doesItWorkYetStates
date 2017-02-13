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

@Autonomous(name="autonomousScrimmage", group="Testing")  // @Autonomous(...) is the other common choice
@Disabled
public class gyroWireImplementation extends LinearOpMode {
    HardwareMapLucyV4 robot;
    Wire sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //robot = new HardwareMapLucyV4();
        //robot.init(hardwareMap);
        //robot.zero(this);
        sensor = new Wire(hardwareMap, "gyro",2*0x10);


        //Wait for start and reset the runtime count
        waitForStart();
        telemetry.addData("Initializing: ", "" );
        telemetry.update();
        sensor.beginWrite(0x03);
        sensor.write(0x4E);
        sensor.endWrite();
        Thread.sleep(200);
        telemetry.addData("Calibrating: ", "");
        telemetry.update();
        calibrate();
        telemetry.addData("Calibrating: ", "Done!");
        telemetry.update();
        int heading = 0;
       while(opModeIsActive()){
           sensor.requestFrom(0x03,1);
           if(sensor.responseCount() > 0){
               sensor.getResponse();
               if(sensor.isRead()){
                   heading = sensor.readHL();
               }
           }
            telemetry.addData("Heading: " , heading);
            telemetry.update();
           idle();
       }

    }

    void calibrate() throws InterruptedException {
        int value;
        while(opModeIsActive()){
            sensor.requestFrom(0x03,1);
            if(sensor.responseCount() > 0){
                sensor.getResponse();
                if(sensor.isRead()){
                   value = sensor.readHL();
                    telemetry.addData("Value: ", value);
                    telemetry.update();
                    if(value == 0x00) return;
                }
            }
            idle();
        }
        return;
    }

}

