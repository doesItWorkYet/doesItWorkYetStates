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

@Autonomous(name="turn test accurate", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class turnTestAccurate extends LinearOpMode {
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
        oneWheelTurnTest(robot.LEFT_MOTOR, 180, .2, .01);

       while(opModeIsActive()){


           idle();
       }

    }
    public void oneWheelTurnTest(int motor, double degree, double power, double error) throws InterruptedException {
        int direction = 0;
        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
        if(degree>0){
            direction = (robot.RIGHT_MOTOR == motor)? -1 : 1;
        }
        if(degree<0){
            direction = (robot.RIGHT_MOTOR == motor)? 1 : -1;
        }
        DcMotor dcMotor = (motor == robot.RIGHT_MOTOR)? robot.rightMotor : robot.leftMotor;
        //dcMotor.setZeroPowerBehavior();
        double trackCircumference = robot.ROBOT_WHEEL_TRACK*Math.PI;
        double distancePerDegree = trackCircumference/360.0;
        double rotations = Math.abs(degree)*distancePerDegree/robot.ROBOT_WHEEL_CIRCUMFERENCE;
        telemetry.addData("Rotation to travel: ", rotations);
        telemetry.update();
        double ticks = rotations * robot.TICKS_PER_REV_ANDYMARK * robot.TURN_CORRECTION_FACTOR;
        int currentPosition = (dcMotor.getCurrentPosition());
        int targetPosition = (int) (currentPosition + (direction * ticks));
        telemetry.addData("Error: ", Math.abs(((double)targetPosition - dcMotor.getCurrentPosition())/robot.TICKS_PER_REV_ANDYMARK));
        telemetry.update();
        while(Math.abs(((double)targetPosition - (double)dcMotor.getCurrentPosition())/(double)robot.TICKS_PER_REV_ANDYMARK) > error) {
            dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dcMotor.setTargetPosition(targetPosition);
            dcMotor.setPower(power);
            while (dcMotor.isBusy()) idle();
            dcMotor.setPower(0);
            if(Math.abs(((double)targetPosition - (double)dcMotor.getCurrentPosition())/(double)robot.TICKS_PER_REV_ANDYMARK) <= error)
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.brakeTemporarily(this);
        }
        telemetry.addData("Error: ", Math.abs((targetPosition - dcMotor.getCurrentPosition())/robot.TICKS_PER_REV_ANDYMARK));
        telemetry.addData("Rotations desired: ", rotations);
        telemetry.addData("Rotations traveled: ", (double)(dcMotor.getCurrentPosition() - currentPosition) / robot.TICKS_PER_REV_ANDYMARK);
        telemetry.update();

    }

}

