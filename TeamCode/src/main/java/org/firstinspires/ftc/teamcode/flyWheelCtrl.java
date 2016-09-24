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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="servo Test", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class flyWheelCtrl extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //DcMotor flyMotor = null;
    Servo clawLeft = null;
    Servo clawRight = null;
    Servo angleLeft = null;
    Servo angleRight = null;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //flyMotor = hardwareMap.dcMotor.get("fly wheel");
        clawLeft = hardwareMap.servo.get("claw Left");
        clawRight = hardwareMap.servo.get("claw Right");
        angleLeft = hardwareMap.servo.get("angle Left");
        angleRight = hardwareMap.servo.get("angle Right");
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        //flyMotor.setPower(0.00);
        clawLeft.setPosition(0.00);
        clawRight.setPosition(1.0);
        angleLeft.setPosition(0.00);
        angleRight.setPosition(1.0);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            //leftMotor.setPower(-gamepad1.left_stick_y);
            //rightMotor.setPower(-gamepad1.right_stick_y);
            //flyMotor.setPower(gamepad1.left_stick_y);
            /*
            if (y > 0) {
                double lmotorpower = y - x;
                rightMotor.setPower(lmotorpower);
            }
            else if (y < 0) {
                double rmotorpower = y - (-x);
                leftMotor.setPower(rmotorpower);
            }
            */
            double clawLPosition = clawLeft.getPosition();
            if (gamepad1.dpad_up) {
                if (clawLPosition < 1.00) {
                    clawLeft.setPosition(clawLPosition + 0.005);
                }
            }
            if (gamepad1.dpad_down) {
                if (clawLPosition > 0.0) {
                    clawLeft.setPosition(clawLPosition - 0.005);
                }
            }
            double clawRPosition = clawRight.getPosition();
            if (gamepad1.dpad_up) {
                if (clawRPosition > 0.0) {
                    clawRight.setPosition(clawRPosition - 0.005);
                }
            }
            if (gamepad1.dpad_down) {
                if (clawRPosition < 1.0) {
                    clawRight.setPosition(clawRPosition + 0.005);
                }
            }

            double angleLPosition = angleLeft.getPosition();
            if (gamepad1.dpad_right) {
                if (angleLPosition < 1.00) {
                    angleLeft.setPosition(angleLPosition + 0.005);
                }
            }
            if (gamepad1.dpad_left) {
                if (angleLPosition > 0.0) {
                    angleLeft.setPosition(angleLPosition - 0.005);
                }
            }
            double angleRPosition = angleRight.getPosition();
            if (gamepad1.dpad_right) {
                if (angleRPosition > 0.0) {
                    angleRight.setPosition(angleRPosition - 0.005);
                }
            }
            if (gamepad1.dpad_left) {
                if (angleRPosition < 1.0) {
                    angleRight.setPosition(angleRPosition + 0.005);
                }
            }
            idle();
            }
            //leftMotor.setPower();
            //rightMotor.setPower(gamepad1.left_stick_y);

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

    public void setDcMotorRPM(DcMotor motor, double revolutions){
        if (revolutions > 120) {
            revolutions = 120;
        }
        else if (revolutions < 0) {
            revolutions = 0;
        }

    }




    }

