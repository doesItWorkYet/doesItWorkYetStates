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
    private ElapsedTime runtime = new ElapsedTime();
    //Declare Motors
    DcMotor driveLeft = null;
    DcMotor driveRight = null;
    DcMotor flyWheel1 = null;
    DcMotor flyWheel2 = null;
    DcMotor sweep = null;
    DcMotor claw = null;
    //Declare Servos
    Servo indexer = null;
    Servo leftButtonPusher = null;
    Servo rightButtonPusher = null;

    //Constant rates of acceleration
    final double ACCELERATION_OF_MAIN_MOTORS = 1;
    final double ACCELERATION_OF_FLY_WHEEL = .25;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Map Motors
        driveLeft = hardwareMap.dcMotor.get("leftMotor");
        driveRight = hardwareMap.dcMotor.get("rightMotor");
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");
        claw = hardwareMap.dcMotor.get("claw");
        sweep = hardwareMap.dcMotor.get("sweep");
        //Map Servos
        indexer = hardwareMap.servo.get("indexer");
        leftButtonPusher = hardwareMap.servo.get("leftButtonPusher");
        rightButtonPusher = hardwareMap.servo.get("rightButtonPusher");

        //Change necessary motor directions
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Ensure no motors begin with power allocated
        driveLeft.setPower(0);
        driveRight.setPower(0);
        sweep.setPower(0);
        claw.setPower(0);

        waitForStart();
        runtime.reset();

        //Declare variables for use in runtime loop
        double timeStartAcceleratingLeftMotor = 0;
        double timeStartAcceleratingRightMotor = 0;
        double timeStartAcceleratingFlyWheel = 0;
        boolean hasLeftBeenAccelerating = false;
        boolean hasRightBeenAccelerating = false;
        boolean hasFlyWheelBeenAccelerating = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //When the right trigger is fully pressed, turn on the flywheel motors
            if(gamepad1.right_trigger == 1){
                flyWheel1.setPower(1);
                flyWheel2.setPower(1);
            }

            //If the right trigger is not fully pressed, ensure the flywheel motors are turned off
            if(gamepad1.right_trigger < 1){
                flyWheel1.setPower(0);
                flyWheel2.setPower(0);
            }


            //Move the indexer into the load position
            if(gamepad1.y){
                indexer.setPosition(40/180);
            }
            //Move the indexer back to default position
            else if(!gamepad1.y) {
                indexer.setPosition(0 / 180);
            }


            if(gamepad1.x){
                //Check to see if the flywheels are currently accelerating
                if(hasFlyWheelBeenAccelerating){
                    //Determine what the current velocity should be based on time since start of accelerate
                    double currentTime = System.currentTimeMillis();
                    double flyWheelPower = determineVelocity(ACCELERATION_OF_FLY_WHEEL,currentTime-timeStartAcceleratingFlyWheel);
                    //Limit the power from being higher than motors can go
                    if(flyWheelPower > 1){
                        flyWheelPower = 1;
                    }
                    //Set power
                    flyWheel1.setPower(flyWheelPower);
                }
                else{
                    //Begin the acceleration of the flywheels and determine time of start
                    hasFlyWheelBeenAccelerating = true;
                    timeStartAcceleratingFlyWheel = System.currentTimeMillis();
                }
            }

            else{
                //The flywheels are not acceleration
                hasFlyWheelBeenAccelerating = false;
            }


            if(gamepad1.left_bumper){
                //Check to see if the left drive motor is currently accelerating
                if(hasLeftBeenAccelerating){
                    //Determine what the current velocity should be based on the time since start of accelerate
                    double currentTime = System.currentTimeMillis();
                    double leftPower = determineVelocity(ACCELERATION_OF_MAIN_MOTORS,currentTime-timeStartAcceleratingLeftMotor);
                    //Limit the power from being higher than the motors can go
                    if(leftPower > 1){
                        leftPower = 1;
                    }
                    //Set power
                    driveLeft.setPower(leftPower);
                }
                else {
                    //Begin the acceleration of the left drive motor and determine time of start
                    hasLeftBeenAccelerating = true;
                    timeStartAcceleratingLeftMotor = System.currentTimeMillis();
                }
            }

            else{
                //The left drive motor is not accelerating
                hasLeftBeenAccelerating = false;
            }

            if(gamepad1.right_bumper){
                //Check to see if the right drive motor is current accelerating
                if(hasRightBeenAccelerating){
                    //Determine what the current velocity should be based on the time since start of accelerate
                    double currentTime = System.currentTimeMillis();
                    double rightPower = determineVelocity(ACCELERATION_OF_MAIN_MOTORS,currentTime-timeStartAcceleratingRightMotor);
                    //Limit the power from going higher than the motor can go
                    if(rightPower > 1){
                        rightPower = 1;
                    }
                    //Set power
                    driveRight.setPower(rightPower);
                }
                else{
                    //Begin the acceleration of the right drive motor and determine the time of start
                    hasRightBeenAccelerating = true;
                    timeStartAcceleratingRightMotor = System.currentTimeMillis();
                }
            }
            else{
                //The right drive motor is not accelerating
                hasRightBeenAccelerating = false;
            }

            if(gamepad1.left_trigger > 0){
                //Check to see if the left drive motor is currently accelerating
                if(hasLeftBeenAccelerating){
                    //Determine what the current velocity should be based on the time since start of accelerate and make it negative
                    double currentTime = System.currentTimeMillis();
                    double leftPower = (determineVelocity(ACCELERATION_OF_MAIN_MOTORS,currentTime-timeStartAcceleratingLeftMotor))*-1;
                    //Limit the power from going lower than the motor can go
                    if(leftPower < -1){
                        leftPower = -1;
                    }
                    //Set power
                    driveLeft.setPower(leftPower);
                }
                else{
                    //Begin the acceleration of the left drive motor and determine the time of start
                    hasLeftBeenAccelerating = true;
                    timeStartAcceleratingLeftMotor = System.currentTimeMillis();
                }
            }
            else{
                //The left drive motor is not accelerating
                hasLeftBeenAccelerating = false;
            }

            if(gamepad1.right_trigger > 0){
                //Check to see if the right drive motor is currently accelerating
                if(hasRightBeenAccelerating){
                    //Determine what the current velocity should be based on the time since start of accelerate and make it negative
                    double currentTime = System.currentTimeMillis();
                    double rightPower = (determineVelocity(ACCELERATION_OF_MAIN_MOTORS,currentTime-timeStartAcceleratingRightMotor))*-1;
                    //Limit the power from going lower than the motor can go
                    if(rightPower < -1){
                        rightPower = -1;
                    }
                    //Set power
                    driveRight.setPower(rightPower);
                }
                else{
                    // Begin the acceleration of the right drive motor and determine the time of start
                    hasRightBeenAccelerating = true;
                    timeStartAcceleratingRightMotor = System.currentTimeMillis();
                }
            }
            else{
                //The right drive motor is not accelerating
                hasRightBeenAccelerating = false;
            }


            //When the A button is pressed, turn the sweep motor in the positive direction
            if(gamepad1.a) {
                sweep.setPower(1);
            }
            //When the B button is pressed, turn the sweep motor in the negative direction
            if(gamepad1.b) {
                sweep.setPower(-1);
            }
            //If neither the A or B button are pressed, turn off the sweep motor
            if(!gamepad1.a & !gamepad1.b) {
                sweep.setPower(0);
            }

            //If the dpad up button is pressed, move the claw motor in the positive direction
            if(gamepad1.dpad_up) {
                claw.setPower(0.1);
            }
            //If the dpad down button is pressed, move the claw motor in the negative direction
            if(gamepad1.dpad_down) {
                claw.setPower(-0.1);
            }
            //If neither the dpad up or down buttons are pressed, turn off the claw motor
            if(!gamepad1.dpad_down && !gamepad1.dpad_up) {
                claw.setPower(0);
            }

            //If the dpad left button is pressed, move the left button pusher to the down position
            if(gamepad1.dpad_left) {
                leftButtonPusher.setPosition(90/180);
            }
            //If the dpad left button is not pressed, move the left button pusher to the standby position
            if(!gamepad1.dpad_left) {
                leftButtonPusher.setPosition(0/180);
            }
            //If the dpad right button is pressed, move the right button pusher to the down position
            if(gamepad1.dpad_right) {
                rightButtonPusher.setPosition(90/180);
            }
            //If the dpad right button is not pressed, move the right button pusher to the standby position
            if(!gamepad1.dpad_right) {
                rightButtonPusher.setPosition(0/180);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }



    }

    public double determineVelocity(double acceleration, double time){
        double velocity = acceleration*time;
        return velocity;
    }
}
