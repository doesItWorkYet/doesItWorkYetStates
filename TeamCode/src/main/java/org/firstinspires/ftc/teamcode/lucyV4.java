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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Lucile User Controlled", group="User Controlled")  // @Autonomous(...) is the other common choice
@Disabled
public class lucyV4 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //Declare Motors
    DcMotor driveLeft = null;
    DcMotor driveRight = null;
    DcMotor flyWheel1 = null;
    DcMotor flyWheel2 = null;
    RPMCounter flyWheel1Counter = null;
    RPMCounter flyWheel2Counter = null;
    motorController driveLeftController = null;
    motorController driveRightController = null;
    DcMotor sweep = null;
    //DcMotor claw = null;
    //Declare Servos
    Servo indexer = null;
    Servo leftButtonPusher = null;
    Servo rightButtonPusher = null;

    //Constants for use in encoders
    final int TICKS_PER_REV_ANDYMARK = 1120;
    final int TICKS_PER_REV_TETRIX = 1440;

    //Constant rates of acceleration
    final double ACCELERATION_OF_MAIN_MOTORS = .1;
    final double ACCELERATION_OF_FLY_WHEEL = .25;
    final double ACCELERATION_EXPONENT = 2;
    final double ACCELERATION_COEFFICIENT = .2;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Map Motors
        driveLeft = hardwareMap.dcMotor.get("leftMotor");
        driveRight = hardwareMap.dcMotor.get("rightMotor");
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");
        driveLeftController = new motorController(driveLeft, ACCELERATION_COEFFICIENT, .1);
        driveRightController = new motorController(driveRight, ACCELERATION_COEFFICIENT, .1);
        flyWheel1Counter = new RPMCounter(flyWheel1, TICKS_PER_REV_TETRIX);
        flyWheel2Counter = new RPMCounter(flyWheel2, TICKS_PER_REV_TETRIX);
        //claw = hardwareMap.dcMotor.get("claw");
        sweep = hardwareMap.dcMotor.get("sweep");
        //Map Servos
        indexer = hardwareMap.servo.get("indexer");
        leftButtonPusher = hardwareMap.servo.get("leftButtonPusher");
        rightButtonPusher = hardwareMap.servo.get("rightButtonPusher");

        //Change necessary motor directions
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Turn on encoders where needed
        //claw.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //claw.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        //Ensure no motors begin with power allocated
        driveLeft.setPower(0);
        driveRight.setPower(0);
        sweep.setPower(0);
        //claw.setPower(0);
        leftButtonPusher.setPosition(180.0);
        rightButtonPusher.setPosition(180.0);

        waitForStart();
        runtime.reset();

        //Declare variables for use in runtime loop
        double timeStartAcceleratingLeftMotor = 0;
        double timeStartAcceleratingRightMotor = 0;
        double timeStartAcceleratingFlyWheel = 0;
        int rps1 = 1;
        int rps2 = 1;
        boolean hasLeftBeenAccelerating = false;
        boolean hasRightBeenAccelerating = false;
        boolean hasFlyWheelBeenAccelerating = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("flyWheel1", "RPS: "+ flyWheel1Counter.getRPS());
            telemetry.addData("flyWheel2", "RPS: "+ flyWheel2Counter.getRPS());
            telemetry.update();

            //When the right trigger is fully pressed, turn on the flywheel motors
            if(gamepad1.x){
                flyWheel1.setPower(rps1);
                flyWheel2.setPower(rps2);
            }

            //If the right trigger is not fully pressed, ensure the flywheel motors are turned off
            if(!gamepad1.x){
                flyWheel1.setPower(0);
                flyWheel2.setPower(0);
            }

            //Move the indexer into the load position
            if(gamepad1.y){
                indexer.setPosition(80.0/360.0);
            }
            //Move the indexer back to default position
            else if(!gamepad1.y) {
                indexer.setPosition(0.0);
            }

            /*
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
            */

/*
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



            //--------------------------------------------------------------------------------------
            /*

            double currentAccel = 0.1;
            double currentPower = 0.0;
            if (gamepad1.dpad_up) {
                currentAccel += 0.001;
            }
            else {
                currentAccel = 0.1;
            }

            if (currentAccel < 1.0 && currentAccel > 0.1) {
                currentPower = currentAccel;
            }

            else if (currentAccel > 1.0 || currentAccel == 1.0) {
                currentPower = 1.0;
            }

            else {
                currentPower = 0.1;
            }

            if (gamepad1.right_bumper) {
                driveRight.setPower(currentPower);
            }
            if (gamepad1.right_trigger > 0.5) {
                driveRight.setPower(-currentPower);
            }
            if (!gamepad1.right_bumper && gamepad1.right_trigger < 0.5) {
                driveRight.setPower(0);
            }

            if (gamepad1.left_bumper) {
                driveLeft.setPower(currentPower);
            }
            if (gamepad1.left_trigger > 0.5) {
                driveLeft.setPower(-currentPower);
            }
            if (!gamepad1.left_bumper && gamepad1.left_trigger < 0.5) {
                driveLeft.setPower(0);
            }
            */

            //--------------------------------------------------------------------------------------

            /*boolean fastOrSlow = false;
            if (gamepad1.dpad_up) {
                fastOrSlow = true;
            }
            if (gamepad1.right_bumper) {
                if (fastOrSlow) {
                    driveRight.setPower(1);
                }
                if (!fastOrSlow) {
                    driveRight.setPower(0.5);
                }
            }
            if (gamepad1.right_trigger > 0) {
                if (fastOrSlow) {
                    driveRight.setPower(-1);
                }
                if (!fastOrSlow) {
                    driveRight.setPower(-0.5);
                }
            }
            if (!gamepad1.right_bumper && gamepad1.right_trigger == 0) {
                driveRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                if (fastOrSlow) {
                    driveLeft.setPower(1);
                }
                if (!fastOrSlow) {
                    driveLeft.setPower(0.5);
                }
            }
            if (gamepad1.left_trigger > 0) {
                if (fastOrSlow) {
                    driveLeft.setPower(-1);
                }
                if (!fastOrSlow) {
                    driveLeft.setPower(-0.5);
                }
            }
            if (!gamepad1.left_bumper && gamepad1.left_trigger == 0) {
                driveLeft.setPower(0);
            }*/

            if(gamepad1.left_trigger > .1){
                driveLeftController.accelationRuntime(true);
            }

            if(gamepad1.right_trigger > .1){
                driveRightController.accelationRuntime(true);
            }

            if(gamepad1.left_bumper){
                driveLeftController.accelationRuntime(false);
            }

            if(gamepad1.right_bumper){
                driveRightController.accelationRuntime(false);
            }

            if(gamepad1.left_trigger <= .1 && !gamepad1.left_bumper){
                driveLeftController.stationaryRuntime();
            }

            if(gamepad1.right_trigger <= .1 && !gamepad1.right_bumper) {
                driveRightController.stationaryRuntime();
            }
            //else driveRightController.stop();
            //When the A button is pressed, turn the sweep motor in the positive direction
            if(gamepad1.a) {
                sweep.setPower(1);
            }
            //When the B button is pressed, turn the sweep motor in the negative direction
            if(gamepad1.b) {
                sweep.setPower(-1);
            }
            //If neither the A or B button are pressed, turn off the sweep motor
            if(!gamepad1.a & !gamepad2.b) {
                sweep.setPower(0);
            }

            //--------------------------------------------------------------------------------------
            /*
            if (gamepad2.dpad_up) {
                claw.setPower(1);
                claw.setTargetPosition(claw.getCurrentPosition() + 10);
            }

            if (gamepad2.dpad_down) {
                claw.setPower(-1);
                claw.setTargetPosition(claw.getCurrentPosition() - 10);
            }
            */
            //--------------------------------------------------------------------------------------
            /*
            //If the dpad up button is pressed, move the claw motor in the positive direction
            if(gamepad2.dpad_up) {
                claw.setPower(1);
            }
            //If the dpad down button is pressed, move the claw motor in the negative direction
            if(gamepad2.dpad_down) {
                claw.setPower(-1);
            }
            //If neither the dpad up or down buttons are pressed, turn off the claw motor
            if(!gamepad2.dpad_down && !gamepad2.dpad_up) {
                claw.setPower(0);
            }
            */
            if(gamepad1.dpad_right){
                rightButtonPusher.setPosition(0.0);
            }
            if(gamepad1.dpad_left){
                leftButtonPusher.setPosition(0.0);
            }
            if(gamepad1.left_stick_button){
                leftButtonPusher.setPosition(180.0);
            }
            if(gamepad1.right_stick_button){
                rightButtonPusher.setPosition(180.0);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public double determineVelocity(double acceleration, double time){
        double velocity = acceleration*time;
        return velocity;
    }

    long oneSecond = 1000;
    class motorController{
        DcMotor motor;
        double accelerationCoefficient;
        double startPower;
        double maxPower = 0.5;
        double currentVelocity;
        long lastAccelerationCall = 0;
        double accelerationExponent = 2;
        boolean decelerating = false;
        int decelerateDirection = 1;
        double decelerateSeconds = 2.0;
        double decelerateStartPower = 0.0;

        public motorController(DcMotor motor, double accelerationCoefficient, double startVelocity){
            this.motor = motor;
            this.accelerationCoefficient = accelerationCoefficient;
            this.startPower = startVelocity;
        }

        public void setPower(double amount) {
            if (amount > maxPower) amount = maxPower;
            if (amount < -maxPower) amount = -maxPower;
            motor.setPower(amount);
        }

        public void setAccelerationExponent(double exponent){
            this.accelerationExponent = exponent;
        }

        public void startAcceleration(){
            decelerating = false;
            lastAccelerationCall = System.currentTimeMillis();
            setPower(startPower);
        }

        public void stop(){
            //setPower(0);
            lastAccelerationCall = 0;
        }

        public void stationaryRuntime() {
            if (!decelerating) {
                lastAccelerationCall = System.currentTimeMillis();
                double motorPower = motor.getPower();
                decelerateStartPower = Math.abs(motorPower);
                decelerateDirection = motorPower < 0 ? -1 : 1;
                decelerating = true;
            }
            else {
                double time = (System.currentTimeMillis() - lastAccelerationCall) / 1000.0;
                double power = decelerateStartPower * (1.0 - (1.0 / decelerateSeconds) * time);
                power = Math.max(0.0, power);
                setPower(decelerateDirection * power);
            }
        }

        public void accelationRuntime(boolean isReversed){
            if(lastAccelerationCall == 0) this.startAcceleration();
            else {
                double time = (System.currentTimeMillis() - lastAccelerationCall) / 1000.0;
                double power = Math.pow(time, accelerationExponent) * accelerationCoefficient + startPower;
                if (power > maxPower) power = maxPower;
                if (isReversed) power *= -1;
                this.motor.setPower(power);
            }
        }

        public void setStartPower(double velocity){
            this.startPower = velocity;
        }

        public void setAccelerationCoefficient(double coefficient){
            this.accelerationCoefficient = coefficient;
        }
    }

    class RPMCounter{
        private DcMotor motor;
        private int ticksPerRevolution = 0;
        private int lastPosition = 0;
        long lastTimeCalled;
        public RPMCounter(DcMotor motor, int ticksPerRevolution){
           this.motor = motor;
            this.ticksPerRevolution = ticksPerRevolution;
            lastTimeCalled = System.currentTimeMillis();
       }

        public int getRPS(){
            long deltaTime = System.currentTimeMillis() - lastTimeCalled;
            long deltaPostion = motor.getCurrentPosition() - lastPosition;
            int ticksPerSecond = (int)(deltaPostion/(double)(deltaTime));
            int rps = (int)(ticksPerSecond/(double)(ticksPerRevolution));
            lastPosition = motor.getCurrentPosition();
            lastTimeCalled = System.currentTimeMillis();
            return rps;
        }
    }

}
