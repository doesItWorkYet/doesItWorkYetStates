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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="encoder Test", group="Testing")  // @Autonomous(...) is the other common choice
@Disabled
public class motorEncoderTrial extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor flyWheelOne = null;
    DcMotor flywheelTwo = null;

    public final int TICKS_PER_REV = 1440;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        flyWheelOne = hardwareMap.dcMotor.get("flyWheel1");
        flywheelTwo = hardwareMap.dcMotor.get("flyWheel2");

        //testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // testMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        //testMotor.setPower(0.00);
        flyWheelOne.setPower(0.00);
        flywheelTwo.setPower(0.00);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
           if(gamepad1.a){
               flyWheelOne.setPower(1);
               flywheelTwo.setPower(-1);
           }
            else if (!gamepad1.a){
               flyWheelOne.setPower(0);
               flywheelTwo.setPower(0);
           }


            //setDcMotorRPM(testMotor, distance.getLightDetected()* 2);

            idle();
            }


        }

    public void moveServo(Servo servo, double degree){
        if (degree > 180) {
            degree = 180;
        }
        else if (degree < 0) {
            degree = 0;
        }
        degree = degree/180;
        servo.setPosition(degree);
    }

    public void setDcMotorRPM(DcMotor motor, double rpm){
        if (rpm > 100) {
            rpm = 100;
        }
        else if (rpm < 0) {
            rpm = 0;
        }
        int rpmToTicksPerMinute = (int) (rpm*TICKS_PER_REV + 0.5);
        motor.setMaxSpeed(rpmToTicksPerMinute);
        motor.setPower(1.0);
    }

    public void stopMotor(DcMotor motor) {
        motor.setPower(0.0);
    }

    public void rotateOnce(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(TICKS_PER_REV);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        while (motor.isBusy()) {

        }
    }

    }

