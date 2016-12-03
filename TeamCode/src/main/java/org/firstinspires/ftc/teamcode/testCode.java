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
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor flyWheel1 = null;
    DcMotor flyWheel2 = null;
    DcMotor scoopLift = null;
    DcMotor scoopTilt = null;
    Servo loadingMechanism = null;

    //Declare constants, if 1 the motor is normal, if -1 the motor should move in a reversed direction
    final double LEFT_MOTOR_INVERSE = -1;
    final double RIGHT_MOTOR_INVERSE = 1;
    final double FLY_WHEEL_1_INVERSE = 1;
    final double FLY_WHEEL_2_INVERSE = -1;
    final double SCOOP_LIFT_INVERSE = 1;
    final double SCOOP_TILT_INVERSE = 1;

    //Declare constant values for use throughout code, affecting speed, default positions, and measurements.
    final double CONTINUOUS_SERVO_NO_ROTATION_POSITION = 0.52;
    final double CONTINUOUS_SERVO_LOAD_POSITION = 0.3;
    final double SCOOP_LIFT_UP_POWER = 0.4;
    final double SCOOP_LIFT_DOWN_POWER = -0.4;
    final double SCOOP_TILT_UP_POWER = -0.3;
    final double SCOOP_TILT_DOWN_POWER = 0.3;
    final double TRIGGER_THRESHOLD = 0.2;
    public final int TICKS_PER_REV = 1440;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //get motors from hardware map
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");
        scoopLift = hardwareMap.dcMotor.get("scoopLift");
        scoopTilt = hardwareMap.dcMotor.get("scoopTilt");

        //get servos from hardware map
        loadingMechanism = hardwareMap.servo.get("loadingMechanism");

        //Set motors to reverse direction if previously declared constant is negative.
        if (LEFT_MOTOR_INVERSE == -1) {
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (RIGHT_MOTOR_INVERSE == -1) {
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (FLY_WHEEL_1_INVERSE == -1) {
            flyWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (FLY_WHEEL_2_INVERSE == -1) {
            flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (SCOOP_LIFT_INVERSE == -1) {
            scoopLift.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (SCOOP_TILT_INVERSE == -1) {
            scoopTilt.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //Set the runtime counter then wait for the code to start.
        double time = runtime.time();
        waitForStart();
        runtime.reset();

        //Ensure no motors or servos are moving or in the wrong position
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        loadingMechanism.setPosition(CONTINUOUS_SERVO_NO_ROTATION_POSITION);

        //Declaring states for use in drive
        boolean leftReverse = false;
        boolean rightReverse = false;

        //Beginning the opMode while loop
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //If the a button is pressed on the first gamepad, turn on the flywheels.
            if (gamepad1.a) {
                flyWheel1.setPower(1);
                flyWheel2.setPower(1);
            }
            //If the a button on the first gamepad is not pressed, turn off the flywheels.
            if (!gamepad1.a) {
                flyWheel1.setPower(0);
                flyWheel2.setPower(0);
            }

            //if the x button on the first gamepad is pressed, begin the loading mechanism rotation
            if(gamepad1.x){
                loadingMechanism.setPosition(CONTINUOUS_SERVO_LOAD_POSITION);
            }
            //if the x buttn is not pressed, set the loading mechanism to not move.
            if(!gamepad1.x){
                loadingMechanism.setPosition(CONTINUOUS_SERVO_NO_ROTATION_POSITION);
            }

            //Declare a variable which is used to control speed
            int gearState = 1;
            //If the b button on the first gamepad is pressed, increment the gear by 1, until it hits 5
            if (gamepad1.b) {
                if (gearState < 5) {
                    gearState += 1;
                }
            }

            //Decide whether or not the triggers are pressed, this is due to the triggers being analog.
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                rightReverse = true;
            }
            else if (gamepad1.right_trigger <= TRIGGER_THRESHOLD) {
                rightReverse = false;
            }
            if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                leftReverse = true;
            }
            else if (gamepad1.left_trigger <= TRIGGER_THRESHOLD) {
                leftReverse = false;
            }

            //If the right bumper is pressed,set the power of the right drive motor to 0.2 times the gear
            if (gamepad1.right_bumper) {
                rightMotor.setPower(0.2 * gearState);
            }
            //If the left bumper is pressed, set the power of the left drive motor to 0.2 times the gear
            if (gamepad1.left_bumper) {
                leftMotor.setPower(0.2 * gearState);
            }
            //If the right trigger is pressed, set the power of the right drive motor to reverse, 0.2 times the gear
            if (rightReverse) {
                rightMotor.setPower(-0.2 * gearState);
            }
            //If the left trigger is pressed, set the power of the left drive motor to reverse, 0.2 times the gear
            if (leftReverse) {
                leftMotor.setPower(-0.2 * gearState);
            }

            //If neither the right bumper or trigger are pressed send no power to the right motor
            if (gamepad1.right_bumper == false && rightReverse == false) {
                rightMotor.setPower(0);
            }
            //If neither the left bumper or trigger are pressed send no power to the right motor
            if (gamepad1.left_bumper == false && leftReverse == false) {
                leftMotor.setPower(0);
            }

            //If the dpad up button on the first gamepad is pressed, set the power of scoopLift equal to its upward constant
            if (gamepad1.dpad_up) {
                scoopLift.setPower(SCOOP_LIFT_UP_POWER);
            }
            //If the dpad down button on the first gamepad is pressed, set the power of scoopLift equal to its downward constant
            if (gamepad1.dpad_down) {
                scoopLift.setPower(SCOOP_LIFT_DOWN_POWER);
            }

            //If neither the dpad up or down on the first gamepad is pressed, set the power of scoopLift to 0.
            if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                scoopLift.setPower(0);
            }

            //If the dpad right button on the first gamepad is pressed, set the power of scoopTilt equal to its upward constant
            if (gamepad1.dpad_right) {
                scoopTilt.setPower(SCOOP_TILT_UP_POWER);
            }
            //If the dpad left button on the first gamepad is pressed, set the power of scoopTilt equal to its downward constant
            if (gamepad1.dpad_left) {
                scoopTilt.setPower(SCOOP_TILT_DOWN_POWER);
            }

            //If neither the dpad left or right buttons on the first gamepad are pressed, set the power of scoopTilt to 0.
            if(!gamepad1.dpad_right & !gamepad1.dpad_left) {
                scoopTilt.setPower(0);
            }

            idle();
            }

        }

    }

