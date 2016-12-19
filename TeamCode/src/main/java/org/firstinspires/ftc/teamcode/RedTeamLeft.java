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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Main Functions", group="User Controlled")  // @Autonomous(...) is the other common choice
@Disabled
public class RedTeamLeft extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //Declare motors
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor flyWheel = null;
    DcMotor sweepMotor = null;
    Servo loadingMechanism = null;
    // note: drive motors have encoders

    final double BOT_WIDTH = 9.75;
    final double WHEEL_CIRCUMFERENCE = 12.44;
    final int TICKS_PER_REV_ANDYMARK = 1120;
    final int TICKS_PER_REV_TETRIX = 1440;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Map motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        flyWheel = hardwareMap.dcMotor.get("flyWheel");
        sweepMotor = hardwareMap.dcMotor.get("sweepMotor");
        loadingMechanism = hardwareMap.servo.get("loadingMechanism");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sweepMotor.setPower(1);
        flyWheel.setPower(0);
        loadingMechanism.setPosition(0);

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();




            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void distance(int distance){

    }

    public void turnDegrees(String direction, int degrees) {
        if (direction == "left") {
            rightMotor.setPower(1);
            leftMotor.setPower(-1);
            double distanceToTravel = 0;
        }
        else if(direction == "right"){
            rightMotor.setPower(-1);
            leftMotor.setPower(1);
            sleepQuietly(2000);
        }
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

    private void sleepQuietly(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            // intentionally empty
        }
    }
}


