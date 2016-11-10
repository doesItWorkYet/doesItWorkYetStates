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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="autonomousLeftStart", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class autonomousLeftStart extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor flyWheel1 = null;
    DcMotor flyWheel2 = null;
    Servo loadingMechanism = null;

    public final int TICKS_PER_REV = 1440;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");
        loadingMechanism = hardwareMap.servo.get("loadingMechanism");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(rightMotor, 48, 5);
        driveForward(leftMotor, 48, 5);
        while (rightMotor.isBusy() && leftMotor.isBusy()) {

        }
        turnReverse(rightMotor, 30, 5);
        while (rightMotor.isBusy()) {

        }
        loadingMechanism.setPosition(1);
        flyWheel1.setPower(1);
        flyWheel2.setPower(1);
        Thread.sleep(5000);
        loadingMechanism.setPosition(0.5);
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);

        turnForward(rightMotor, 50, 5);
        while (rightMotor.isBusy()) {

        }

        driveForward(rightMotor, 30, 5);
        driveForward(leftMotor, 30, 5);




        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();



            idle();
            }


        }

    public void driveForward(DcMotor motor, double distance, double speed) {
        //diameter 4.035 inches circumfirance 12.68 inches
        motor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double x = distance / 12.68;
        int z = (int) x;
        x = x * 1440;
        motor.setTargetPosition(motor.getCurrentPosition() + z);
        if (speed > 5) {
            speed = 5;
        }
        if (speed < 0) {
            speed = 0;
        }
        motor.setPower(speed * 0.2);
    }

    public void turnForward(DcMotor motor, double angle, double speed) {
        double x = angle/360;
        x=x*Math.PI*17;
        driveForward(motor, x, speed);
    }

    public void turnReverse(DcMotor motor, double angle, double speed) {
        double x = angle/360;
        x=x*Math.PI*17;
        driveReverse(motor, x, speed);
    }

    public void driveReverse(DcMotor motor, double distance, double speed) {
        //diameter 4.035 inches circumfirance 12.68 inches
        motor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double x = distance / 12.68;
        int z = (int) x;
        x = x * 1440;
        motor.setTargetPosition(motor.getCurrentPosition() - z);
        if (speed > 5) {
            speed = 5;
        }
        if (speed < 0) {
            speed = 0;
        }
        motor.setPower(-speed * 0.2);
    }


    }

