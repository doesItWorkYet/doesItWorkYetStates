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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autonomousScrimmage", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class autonomousScrimmage extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    //Declare constants, if 1 motor is normal, if -1 reverse motor direciton
    final double LEFT_MOTOR_INVERSE = -1;
    final double RIGHT_MOTOR_INVERSE = 1;

    //Declare constants, contain measurements pertaining to the bot
    final double WHEEL_CIRCUMFERENCE = 12.44;
    final int TICKS_PER_REV_ANDYMARK = 1120;
    final int TICKS_PER_REV_TETRIX = 1440;
    final double BOT_WIDTH = 17;

    //Begin OpMode
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait for start and reset the runtime count
        waitForStart();
        runtime.reset();

        //Drive forward 48 inches
        driveForward(rightMotor, 48, 5);
        driveForward(leftMotor, 48, 5);

        }

    //Function to have a drive motor move forward a distance in inches at a speed, ranging from 1 to 5
    public void driveForward(DcMotor motor, double distance, double speed) {
        //Reset the encoders and ensure the motor is in run to position mode
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Find the number of revolutions needed to achieve distance
        double x = distance / WHEEL_CIRCUMFERENCE;
        //Find the number of ticks needed to achieve distance
        int dist = (int) x * TICKS_PER_REV_ANDYMARK;
        //Set the target position
        motor.setTargetPosition(motor.getCurrentPosition() + dist);
        //Filter out speed values which are too large or small
        if (speed > 5) {
            speed = 5;
        }
        if (speed < 0) {
            speed = 0;
        }
        //Begin motor movement
        motor.setPower(speed * 0.2);
    }

    //Function to have a drive motor move back a distance in inches at a speed, ranging from 1 to 5
    public void driveReverse(DcMotor motor, double distance, double speed) {
        //Reset the encoders and ensure the motor is in run to position mode\
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Find the number of revolutions needed to achieve distance
        double x = distance / WHEEL_CIRCUMFERENCE;
        //Find the number of ticks needed to achieve distance
        int dist = (int) x * TICKS_PER_REV_ANDYMARK;
        //Set the target position
        motor.setTargetPosition(motor.getCurrentPosition() - dist);
        //Filter out speed values which are too large or small
        if (speed > 5) {
            speed = 5;
        }
        if (speed < 0) {
            speed = 0;
        }
        //Begin motor movement
        motor.setPower(-speed * 0.2);
    }

    public void turnForward(DcMotor motor, double angle, double speed) {
        double x = angle/360;
        x=x*Math.PI*BOT_WIDTH;
        driveForward(motor, x, speed);
    }

    public void turnReverse(DcMotor motor, double angle, double speed) {
        double x = angle/360;
        x=x*Math.PI*BOT_WIDTH;
        driveReverse(motor, x, speed);
    }

    }

