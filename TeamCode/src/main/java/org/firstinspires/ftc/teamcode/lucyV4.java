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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Lucile User Controlled", group="User Controlled")  // @Autonomous(...) is the other common choice
//@Disabled
public class lucyV4 extends LinearOpMode {

    /* Declare OpMode members. */
    final double ACCELERATION_OF_MAIN_MOTORS = 1;
    final double ACCELERATION_OF_FLY_WHEEL = .25;
    private ElapsedTime runtime = new ElapsedTime();
    //Declare motors
    DcMotor driveLeft = null;
    DcMotor driveRight = null;
    DcMotor flyWheel1 = null;
    DcMotor flyWheel2 = null;
    DcMotor sweep = null;
    Servo indexer = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Map motors
        driveLeft = hardwareMap.dcMotor.get("leftMotor");
        driveRight = hardwareMap.dcMotor.get("rightMotor");
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");
        sweep = hardwareMap.dcMotor.get("vaccuum");
        indexer = hardwareMap.servo.get("indexer");
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeft.setPower(0);
        driveRight.setPower(0);
        sweep.setPower(0);

        waitForStart();
        runtime.reset();

        double timeStartAcceleratingLeftMotor = 0;
        double timeStartAcceleratingRightMotor = 0;
        double timeStartAcceleratingFlyWheel = 0;
        boolean hasLeftBeenAccelerating = false;
        boolean hasRightBeenAccelerating = false;
        boolean hasFlyWheelBeenAccelerating = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Controller ", "Left Joystick: "+gamepad1.left_stick_y);
            telemetry.addData("Controller", "Right Trigger: "+gamepad1.right_trigger);
            telemetry.addData("Controller", "Right Bumper: "+gamepad1.right_bumper);
            telemetry.addData("Controller", "Left Trigger: "+gamepad1.left_trigger);
            telemetry.addData("Controller", "Left Bumper: "+gamepad1.left_bumper);
            telemetry.update();

            if(gamepad1.right_trigger >= 1){
                flyWheel1.setPower(1);
                flyWheel2.setPower(1);
            }

            if(gamepad1.right_trigger < 1){
                flyWheel1.setPower(0);
                flyWheel2.setPower(0);
            }

            if(gamepad1.y){
                indexer.setPosition(40/180);
            }
            else if(gamepad1.y != true){
                indexer.setPosition(0/180);
            }

            /*if(gamepad1.right_bumper){
                rightPower = 1;
            }
            else if(gamepad1.right_bumper != true){
                rightPower = 0;
            }

            if(gamepad1.left_bumper){
                leftPower = 1;
            }
            else if(gamepad1.left_bumper != true){
                leftPower = 0;
            }*/


            if(gamepad1.x){
                // check if accel is on
                if(hasFlyWheelBeenAccelerating){
                    // determine acceleration
                    double currentTime = System.currentTimeMillis();
                    double flyWheelPower = determineVelocity(ACCELERATION_OF_FLY_WHEEL,currentTime-timeStartAcceleratingFlyWheel);
                    if(flyWheelPower > 1){
                        flyWheelPower = 1;
                    }
                    flyWheel1.setPower(flyWheelPower);
                }
                else{
                    // turn on accel and add time stamp
                    hasFlyWheelBeenAccelerating = true;
                    timeStartAcceleratingFlyWheel = System.currentTimeMillis();
                }
            }
            else{
                // turn off accel
                hasFlyWheelBeenAccelerating = false;
            }

            if(gamepad1.left_bumper){
                // check if accel is on
                if(hasLeftBeenAccelerating){
                    // determine acceleration power
                    double currentTime = System.currentTimeMillis();
                    double leftPower = determineVelocity(ACCELERATION_OF_MAIN_MOTORS,currentTime-timeStartAcceleratingLeftMotor);
                    if(leftPower > 1){
                        leftPower = 1;
                    }
                    driveLeft.setPower(leftPower);
                }
                else {
                    // turn on left accel and add time stamp
                    hasLeftBeenAccelerating = true;
                    timeStartAcceleratingLeftMotor = System.currentTimeMillis();
                }
            }
            else{
                // turn left accel off
                hasLeftBeenAccelerating = false;
            }
            if(gamepad1.right_bumper){
                // check if accel is on
                if(hasRightBeenAccelerating){
                    // determine acceleration power
                    double currentTime = System.currentTimeMillis();
                    double rightPower = determineVelocity(ACCELERATION_OF_MAIN_MOTORS,currentTime-timeStartAcceleratingRightMotor);
                    if(rightPower > 1){
                        rightPower = 1;
                    }
                    driveRight.setPower(rightPower);
                }
                else{
                    // turn on right accel and add time stamp
                    hasRightBeenAccelerating = true;
                    timeStartAcceleratingRightMotor = System.currentTimeMillis();
                }
            }
            else{
                // turn right accel off
                hasRightBeenAccelerating = false;
            }
            if(gamepad1.left_trigger>0){
                // check if accel is on
                if(hasLeftBeenAccelerating){
                    // determine acceleration power
                    double currentTime = System.currentTimeMillis();
                    double leftPower = (determineVelocity(ACCELERATION_OF_MAIN_MOTORS,currentTime-timeStartAcceleratingLeftMotor))*-1;
                    if(leftPower > -1){
                        leftPower = -1;
                    }
                    driveLeft.setPower(leftPower);
                }
                else{
                    // turn on accel and add time stamp
                    hasLeftBeenAccelerating = true;
                    timeStartAcceleratingLeftMotor = System.currentTimeMillis();
                }
            }
            else{
                // turn off accel
                hasLeftBeenAccelerating = false;
            }
            if(gamepad1.right_trigger>0){
                // check if accel is on
                if(hasRightBeenAccelerating){
                    // determine acceleration power
                    double currentTime = System.currentTimeMillis();
                    double rightPower = (determineVelocity(ACCELERATION_OF_MAIN_MOTORS,currentTime-timeStartAcceleratingRightMotor))*-1;
                    if(rightPower>-1){
                        rightPower = -1;
                    }
                    driveRight.setPower(rightPower);
                }
                else{
                    // turn on accel and add time stamp
                    hasRightBeenAccelerating = true;
                    timeStartAcceleratingRightMotor = System.currentTimeMillis();
                }
            }
            else{
                // turn off accel
                hasRightBeenAccelerating = false;
            }



            if(gamepad1.a)
                sweep.setPower(1);
            if(gamepad1.b)
                sweep.setPower(-1);
            if(!gamepad1.a & !gamepad1.b)
                sweep.setPower(0);

            //driveLeft.setPower(leftPower);
            //driveRight.setPower(rightPower);
            //flyWheel1.setPower(1);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }



    }

    public double determineVelocity(double acceleration, double time){
        double velocity = acceleration*time;
        return velocity;
    }
    /*public void accelerationRight(boolean hasRightBeenAccelerating, double timeStartAccelerationRight){
        // right forward
        if(gamepad1.right_bumper){
            // check if accel is on
            if(hasRightBeenAccelerating){

            }
        }
    }
    public void accelerationLeft(){

    }
    public void accelerationFlyWheel(){

    }*/
}
