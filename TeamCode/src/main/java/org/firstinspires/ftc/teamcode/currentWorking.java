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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="currentWorking", group="Testing")  // @Autonomous(...) is the other common choice
@Disabled
public class currentWorking extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor flyWheel1 = null;
    DcMotor flyWheel2 = null;
    public final int TICKS_PER_REV = 1440;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        flyWheel2.setDirection(DcMotor.Direction.REVERSE);
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */


        waitForStart();
        runtime.reset();
        boolean rightReverse = false;
        boolean leftReverse = false;
        setDcMotorRPM(leftMotor, 0);
        setDcMotorRPM(rightMotor, 0);
        setDcMotorRPM(flyWheel1, 0);
        setDcMotorRPM(flyWheel2, 0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Bumper Status", "Right: " + gamepad1.right_bumper + "     Left: " + gamepad1.left_bumper);
            telemetry.addData("Booleans", "Right: " + rightReverse + "     Left: " + leftReverse);
            telemetry.update();

            int gearState = 1;
            if (gamepad1.dpad_up) {
                if (gearState < 5) {
                    gearState += 1;

                }
            }
            if (gamepad1.dpad_down) {
                if (gearState > 1) {
                    gearState -= 1;
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
            if (gamepad1.b) {
                flyWheel2.setPower(0.25);
                flyWheel1.setPower(0.25);
            }
            else if (!gamepad1.b) {
                flyWheel1.setPower(0.0);
                flyWheel2.setPower(0.0);
            }
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

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
