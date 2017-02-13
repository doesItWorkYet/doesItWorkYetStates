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

@TeleOp(name="Motor Proportional Controller Test", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class motorProportionalControllerTest extends LinearOpMode {
    HardwareMapLucyV4 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero(this);
//        MotorProportionalController rightMotor = new MotorProportionalController(robot.rightMotor, robot.ANDY_MARK_PPR*2, 3, robot.TURNING_P);
//        MotorProportionalController leftMotor = new MotorProportionalController(robot.leftMotor, robot.ANDY_MARK_PPR*2, 3, robot.TURNING_P);
        MotorProportionalController flyWheel1 = new MotorProportionalController(robot.flyWheel1, (long)robot.ANDY_MARK_PPR*2, 25, robot.TURNING_P);
        MotorProportionalController flyWheel2 = new MotorProportionalController(robot.flyWheel2, (long)robot.ANDY_MARK_PPR*2, 25, robot.TURNING_P);
        //Wait for start and reset the runtime count
        double flyWheelPower = 0;
        waitForStart();
        while(opModeIsActive()){
            flyWheel1.update();
            flyWheel2.update();
//            rightMotor.update();
//            leftMotor.update();
//            telemetry.addData("Left Motor RPS: ", leftMotor.rpsCounter.getRps());
//            telemetry.addData("Right Motor RPS: ", rightMotor.rpsCounter.getRps());
            telemetry.addData("Fly Wheel 1 RPS: ", flyWheel1.rpsCounter.getRps());
            telemetry.addData("Fly Wheel 2 RPS: ", flyWheel2.rpsCounter.getRps());
            telemetry.update();
//            if(gamepad1.right_trigger > 0.8){
//                rightMotor.setPower(0.5);
//            }
//            if(gamepad1.left_trigger > 0.8){
//                leftMotor.setPower(0.5);
//            }
            if(gamepad1.dpad_up){
                flyWheelPower += 0.25;
                if(flyWheelPower>=1)flyWheelPower = 1;
            }
            if(gamepad1.dpad_down){
                flyWheelPower -= 0.25;
                if(flyWheelPower<=0)flyWheelPower = 0;
            }
            if(gamepad1.x){
                flyWheel1.setPower(flyWheelPower);
                flyWheel2.setPower(flyWheelPower);
            }
            if(!gamepad1.x){
                flyWheel1.setPower(0);
                flyWheel2.setPower(0);
            }

        }
    }

}

