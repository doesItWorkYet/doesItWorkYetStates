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

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="testCode", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class testCode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor flyWheel1 = null;
    DcMotor flyWheel2 = null;
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor extendOTron1 = null;
    DcMotor extendOTron2 = null;
    DcMotor scoopLift = null;
    Servo loadingMechanism = null;
    Servo pushMechanism = null;
    Servo scoopLeft = null;
    Servo scoopRight = null;
    TouchSensor extendOTronLimit1 = null;
    TouchSensor extendOTronLimit2 = null;
    TouchSensor scoopLiftLimit = null;


    public final int TICKS_PER_REV = 1440;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        extendOTron1 = hardwareMap.dcMotor.get("extendOTron1");
        extendOTron2 = hardwareMap.dcMotor.get("extendOTron2");
        scoopLift = hardwareMap.dcMotor.get("scoopLift");
        loadingMechanism = hardwareMap.servo.get("loadingMechanism");
        pushMechanism = hardwareMap.servo.get("pushMechanism");
        scoopLeft = hardwareMap.servo.get("scoopLeft");
        scoopRight = hardwareMap.servo.get("scoopRight");
        extendOTronLimit1 = hardwareMap.touchSensor.get("extendOTronLimit1");
        extendOTronLimit2 = hardwareMap.touchSensor.get("extendOTronLimit2");
        scoopLiftLimit = hardwareMap.touchSensor.get("scoopLiftLimit");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        double time = runtime.time();
        waitForStart();
        runtime.reset();
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);
        loadingMechanism.setPosition(0.5);
        boolean leftReverse = false;
        boolean rightReverse = false;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            while (gamepad1.a) {
                flyWheel1.setPower(1);
                flyWheel2.setPower(1);
                loadingMechanism.setPosition(1);
            }
            if (!gamepad1.a) {
                flyWheel1.setPower(0);
                flyWheel2.setPower(0);
                loadingMechanism.setPosition(0.5);
            }


            int gearState = 1;
            if (gamepad1.b) {
                if (gearState < 5) {
                    gearState += 1;

                }
            }

            if (gamepad1.right_trigger > .2) {
                rightReverse = true;
            }
            else if (gamepad1.right_trigger <= .2) {
                rightReverse = false;
            }
            if (gamepad1.left_trigger > .2) {
                leftReverse = true;
            }
            else if (gamepad1.left_trigger <= .2) {
                leftReverse = false;
            }
            if (gamepad1.right_bumper) {
                rightMotor.setPower(0.2 * gearState);
            }
            if (gamepad1.left_bumper) {
                leftMotor.setPower(0.2 * gearState);
            }
            if (rightReverse) {
                rightMotor.setPower(-0.2 * gearState);
            }
            if (leftReverse) {
                leftMotor.setPower(-0.2 * gearState);
            }

            if (gamepad1.right_bumper == false && rightReverse == false) {
                rightMotor.setPower(0);
            }
            if (gamepad1.left_bumper == false && leftReverse == false) {
                leftMotor.setPower(0);
            }

            if (gamepad2.dpad_left) {
                if (!extendOTronLimit1.isPressed()) {
                    extendOTron1.setPower(1);
                }
                if (!extendOTronLimit2.isPressed()) {
                    extendOTron2.setPower(1);
                }
            }

            if (gamepad2.dpad_right) {
                extendOTron1.setPower(-1);
                extendOTron2.setPower(-1);
            }

            if (gamepad2.b) {
                pushMechanism.setPosition(1);
            }
            if (!gamepad2.b) {
                pushMechanism.setPosition(0);
            }

            while (gamepad1.dpad_up) {
                scoopLeft.setPosition(scoopLeft.getPosition() - .001);
                scoopRight.setPosition(scoopRight.getPosition() + .001);
            }

            while (gamepad1.dpad_down) {
                scoopLeft.setPosition(scoopLeft.getPosition() + .001);
                scoopRight.setPosition(scoopRight.getPosition() - .001);
            }

            while (gamepad1.dpad_right) {
                if (!scoopLiftLimit.isPressed()) {
                    scoopLift.setPower(1);
                }
            }

            while (gamepad1.dpad_left) {
                scoopLift.setPower(-1);
            }


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

    public void accelerateTo(DcMotor motor, double speed) {
        for (double i = motor.getPower(); i < speed; i += 0.01 ) {
            motor.setPower(i);
        }
    }

    public void deccelerateTo(DcMotor motor, double speed) {
        for (double i = motor.getPower(); i > speed; i -= 0.01) {
            motor.setPower(i);
        }
    }

    }

