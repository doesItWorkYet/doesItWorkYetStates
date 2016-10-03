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

@TeleOp(name="drive", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled
public class Drive extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Servo steering = null;
    DcMotor driveMotor = null;
    Servo leftClaw = null;
    Servo rightClaw = null;
    Servo rightScoop = null;
    Servo leftScoop = null;
    DcMotor liftScoop = null;
    DcMotor launcher = null;
    DcMotor extendOMatic = null;
    public final int TICKS_PER_REV = 1440;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        steering = hardwareMap.servo.get("steering");
        driveMotor = hardwareMap.dcMotor.get("driveMotor");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftScoop = hardwareMap.servo.get("leftScoop");
        rightScoop = hardwareMap.servo.get("rightScoop");
        liftScoop = hardwareMap.dcMotor.get("liftScoop");
        launcher = hardwareMap.dcMotor.get("launcher");
        extendOMatic = hardwareMap.dcMotor.get("extendOMatic");
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
        setDcMotorRPM(liftScoop, 0.0);
        setDcMotorRPM(launcher, 0.0);
        setDcMotorRPM(extendOMatic, 0.0);
        setDcMotorRPM(driveMotor, 0.0);
        moveServo(steering, 90.0);
        moveServo(leftClaw, 0.0);
        moveServo(rightClaw, 180.0);
        moveServo(leftScoop, 0.0);
        moveServo(rightScoop, 180.0);
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

            double driveRPM = 0.0;
            if (gamepad1.left_bumper) {
                driveRPM += 5.0;
            }
            if (gamepad1.right_bumper) {
                driveRPM -= 5.0;
            }
            setDcMotorRPM(driveMotor, driveRPM);

            if (gamepad1.a) {
                setDcMotorRPM(driveMotor, 0.0);
            }


            double clawLPosition = leftClaw.getPosition();
            if (gamepad1.dpad_up) {
                if (clawLPosition < 1.00) {
                    leftClaw.setPosition(clawLPosition + 0.005);
                }
            }
            if (gamepad1.dpad_down) {
                if (clawLPosition > 0.0) {
                    leftClaw.setPosition(clawLPosition - 0.005);
                }
            }
            double clawRPosition = rightClaw.getPosition();
            if (gamepad1.dpad_up) {
                if (clawRPosition > 0.0) {
                    rightClaw.setPosition(clawRPosition - 0.005);
                }
            }
            if (gamepad1.dpad_down) {
                if (clawRPosition < 1.0) {
                    rightClaw.setPosition(clawRPosition + 0.005);
                }
            }

            double scoopLPosition = leftScoop.getPosition();
            if (gamepad1.dpad_left) {
                if (scoopLPosition < 1.00) {
                    leftScoop.setPosition(scoopLPosition + 0.005);
                }
            }
            if (gamepad1.dpad_right) {
                if (scoopLPosition > 0.0) {
                    leftScoop.setPosition(scoopLPosition - 0.005);
                }
            }
            double scoopRPosition = rightScoop.getPosition();
            if (gamepad1.dpad_left) {
                if (scoopRPosition > 0.0) {
                    rightScoop.setPosition(scoopRPosition - 0.005);
                }
            }
            if (gamepad1.dpad_right) {
                if (scoopRPosition < 1.0) {
                    rightScoop.setPosition(scoopRPosition + 0.005);
                }
            }

            boolean launcherState = false;

            if (launcherState == false) {
                if (gamepad1.x) {
                    setDcMotorRPM(launcher, 100.0);
                    launcherState = true;
                }
            }
            if (launcherState == true) {
                if (gamepad1.x) {
                    setDcMotorRPM(launcher, 0.0);
                    launcherState = false;
                }
            }

            boolean extendState1 = false;
            boolean extendState2 = false;

            if (extendState1 == false & extendState2 == false) {
                if (gamepad1.b) {
                    setDcMotorRPM(extendOMatic, 50);
                    wait(1000);
                    setDcMotorRPM(extendOMatic, 0);
                    extendState1 = true;
                }
            }
            if (extendState1 == true) {
                if (gamepad1.b) {
                    setDcMotorRPMReverse(extendOMatic, 50);
                    wait(1000);
                    setDcMotorRPM(extendOMatic, 0);
                    extendState1 = false;
                }
            }

            if (extendState2 == false & extendState1 == false) {
                if (gamepad1.y) {
                    setDcMotorRPM(extendOMatic, 50);
                    wait(2000);
                    setDcMotorRPM(extendOMatic, 0);
                    extendState2 = true;
                }
            }
            if (extendState2 == true) {
                if (gamepad1.y) {
                    setDcMotorRPMReverse(extendOMatic, 50);
                    wait(2000);
                    setDcMotorRPM(extendOMatic, 0);
                    extendState2 = false;
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

    public void setDcMotorRPM(DcMotor motor, double rpm){
        if (rpm > 120) {
            rpm = 120;
        }
        else if (rpm < 0) {
            rpm = 0;
        }
        int rpmToTicksPerMinute = (int) (rpm*TICKS_PER_REV + 0.5);
        motor.setMaxSpeed(rpmToTicksPerMinute);
        motor.setPower(1.0);
    }

    public void setDcMotorRPMReverse(DcMotor motor, double rpm){
        if (rpm > 120) {
            rpm = 120;
        }
        else if (rpm < 0) {
            rpm = 0;
        }
        int rpmToTicksPerMinute = (int) (rpm*TICKS_PER_REV + 0.5);
        motor.setMaxSpeed(rpmToTicksPerMinute);
        motor.setPower(-1.0);
    }




    }

