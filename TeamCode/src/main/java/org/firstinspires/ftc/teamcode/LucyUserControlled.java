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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Lucile User Controlled", group="User Controlled")  // @Autonomous(...) is the other common choice
//@Disabled
public class LucyUserControlled extends LinearOpMode {
    HardwareMapLucyV4 robot;
    boolean slowModeToggle = false;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private motorController driveLeftController, driveRightController;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Hello World!");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        telemetry.addData("Status", " hw map init");
        telemetry.update();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Robot init");
        telemetry.update();
        robot.zero(this);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.flyWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        telemetry.addData("Status", "Robot zeros");
        telemetry.update();
        driveLeftController = new motorController(robot.leftMotor,robot.ACCELERATION_COEFFICIENT,robot.START_VELOCITY);
        driveRightController = new motorController(robot.rightMotor,robot.ACCELERATION_COEFFICIENT,robot.START_VELOCITY);
        //RPMCounter flyWheel1Counter = new RPMCounter(robot.flyWheel1, robot.TICKS_PER_REV_ANDYMARK);
        //RPMCounter flyWheel2Counter = new RPMCounter(robot.flyWheel2, robot.TICKS_PER_REV_ANDYMARK);
        waitForStart();
        runtime.reset();

        //Declare variables for use in runtime loop
        RpsCounter flyWheelCounter1 = new RpsCounter(robot.flyWheel1, 28*2, 200);
        RpsCounter flyWheelCounter2 = new RpsCounter(robot.flyWheel2, 28*2, 200);
//        RpsCounter rightMotorCounter = new RpsCounter(robot.rightMotor, robot.TICKS_PER_REV_ANDYMARK);
//        RpsCounter leftMotorCounter = new RpsCounter(robot.leftMotor, robot.TICKS_PER_REV_ANDYMARK);
//        RpsCounter rightSpoolCounter = new RpsCounter(robot.extendotronRight, robot.TICKS_PER_REV_ANDYMARK);
//        RpsCounter leftSpoolCounter = new RpsCounter(robot.extendotronLeft, robot.TICKS_PER_REV_ANDYMARK);
//        RpsCounter sweepCounter = new RpsCounter(robot.sweep, robot.TICKS_PER_REV_ANDYMARK);
        double rps1 = 1;
        double rps2 = 1;
        double flyWheelPower = 1;
        int flyWheelDeflectorPosition = 1;
        int flyWheelDeflectorPosition2 = 1;
        boolean isGoingUp = false;
        boolean sweepForwardOnOrOff = false;
        boolean sweepReverseOnOrOff = false;
        boolean driveDirection = false;
        boolean armletsDeployed = false;
        boolean samDriveMode = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flywheel1 RPS: ", flyWheelCounter1.getRps());
            telemetry.addData("Flywheel2 RPS: ", flyWheelCounter2.getRps());
            //telemetry.addData("Right Motor RPS: ", rightMotorCounter.getRps());
            //telemetry.addData("Left Motor RPS: ", leftMotorCounter.getRps());
            telemetry.addData("Deflector Position: ", flyWheelDeflectorPosition);
            telemetry.addData("Heading: ", robot.gyro.getIntegratedZValue());
            flyWheelCounter1.update();
            flyWheelCounter2.update();


            if(gamepad2.a){
                flyWheelDeflectorPosition++;
                if (flyWheelDeflectorPosition > 3) { flyWheelDeflectorPosition = 3; }
                robot.setTargetDistanceForParticleShooter(flyWheelDeflectorPosition, this);
                while(gamepad2.a && !robot.safety(this));
            }
            if(gamepad2.b){
                flyWheelDeflectorPosition--;
                if (flyWheelDeflectorPosition < -3) { flyWheelDeflectorPosition = -3; }
                robot.setTargetDistanceForParticleShooter(flyWheelDeflectorPosition, this);
                while(gamepad2.b);
            }


            //Move the indexer into the load position
            if(gamepad2.y){
                robot.indexer.setPosition(robot.INDEXER_FIRE_POSITION/180.0);
            }
            //Move the indexer back to default position
            else if(!gamepad2.y) {
                robot.indexer.setPosition(robot.INDEXER_LOAD_POSITION/180.0);
            }

            //Deploy the armlets
            if(gamepad2.dpad_up){
                robot.deployArmlets();
            }
            //Store the armlets
            if(gamepad2.dpad_down){
                robot.storeArmlets();
            }

            //Drive Left Motor in Reverse
            if(samDriveMode){
                if(gamepad1.left_trigger > .1){
                    if(slowModeToggle) robot.rightMotor.setPower(robot.MAIN_MOTOR_MAX_POWER/2.0);
                    else driveRightController.accelationRuntime(true);
                }
                if(gamepad1.right_trigger > .1){
                    if(slowModeToggle) robot.leftMotor.setPower(robot.MAIN_MOTOR_MAX_POWER/2.0);
                    else driveLeftController.accelationRuntime(true);
                }
                if(gamepad1.left_bumper){
                    if(slowModeToggle) robot.rightMotor.setPower(-robot.MAIN_MOTOR_MAX_POWER/2.0);
                    else driveRightController.accelationRuntime(false);
                }
                if(gamepad1.right_bumper){
                    if(slowModeToggle) robot.leftMotor.setPower(-robot.MAIN_MOTOR_MAX_POWER/2.0);
                    else driveLeftController.accelationRuntime(false);
                }
                //If neither the left trigger or bumper is pressed, the left motor should be stationary
                if(gamepad1.left_trigger <= .1 && !gamepad1.left_bumper){
                    driveRightController.stationaryRuntime();
                }
                //If neither the right trigger or bumper is pressed, the right motor should be stationary
                if(gamepad1.right_trigger <= .1 && !gamepad1.right_bumper) {
                    driveLeftController.stationaryRuntime();
                }
                while(gamepad1.dpad_left){
                    idle();
                    samDriveMode = false;
                }
            }
            if(!samDriveMode) {
//                if (gamepad1.left_trigger > .1) {
//                    if (slowModeToggle) robot.leftMotor.setPower(robot.MAIN_MOTOR_MAX_POWER / 2.0);
//                    else driveLeftController.accelationRuntime(true);
//                }
//                //Drive Right Motor in Reverse
//                if (gamepad1.right_trigger > .1) {
//                    if (slowModeToggle) robot.rightMotor.setPower(robot.MAIN_MOTOR_MAX_POWER / 2.0);
//                    else driveRightController.accelationRuntime(true);
//                }
//                //Drive Left Motor Forward
//                if (gamepad1.left_bumper) {
//                    if (slowModeToggle) robot.leftMotor.setPower(-robot.MAIN_MOTOR_MAX_POWER / 2.0);
//                    else driveLeftController.accelationRuntime(false);
//                }
//                //Drive Right Motor Forward
//                if (gamepad1.right_bumper) {
//                    if (slowModeToggle) robot.rightMotor.setPower(-robot.MAIN_MOTOR_MAX_POWER / 2.0);
//                    else driveRightController.accelationRuntime(false);
//                }
//                //If neither the left trigger or bumper is pressed, the left motor should be stationary
//                if(gamepad1.left_trigger <= .1 && !gamepad1.left_bumper){
//                    driveLeftController.stationaryRuntime();
//                }
//                //If neither the right trigger or bumper is pressed, the right motor should be stationary
//                if(gamepad1.right_trigger <= .1 && !gamepad1.right_bumper) {
//                    driveRightController.stationaryRuntime();
//                }
                if(gamepad1.left_stick_y > 0.8){
                    driveLeftController.accelationRuntime(true);
                    driveRightController.accelationRuntime(true);
                }
                if(gamepad1.left_stick_y < -0.8){
                    driveLeftController.accelationRuntime(false);
                    driveRightController.accelationRuntime(false);
                }
                if(gamepad1.right_stick_x > 0.8){
                    driveLeftController.maxPower = driveRightController.maxPower - 0.3;
                }
                if(gamepad1.right_stick_x < -0.8){
                    driveRightController.maxPower = driveLeftController.maxPower - 0.3;
                }
                while (gamepad1.dpad_left) {
                    idle();
                    samDriveMode = true;
                }
            }


            //When the A button is pressed, turn the sweep motor in the positive direction
            if(gamepad1.a) {
                robot.sweep.setPower(1);
            }
            //When the B button is pressed, turn the sweep motor in the negative direction
            if(gamepad1.b) {
                robot.sweep.setPower(-1);
            }
            //If neither the A or B button are pressed, turn off the sweep motor
            if(!gamepad1.a & !gamepad1.b) {
                robot.sweep.setPower(0);
            }
            //Move slower
            if(gamepad1.y){
                slowModeToggle = true;
                while(gamepad1.y);
            }
            //Deploy the beacon presser
            if(gamepad1.dpad_up){
                robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_PRESS_POSITION/180.0);
                robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_PRESS_POSITION/180.0);
            }
            //Store the beacon pesser
            if(gamepad1.dpad_down){
                robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
                robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
            }

            if(gamepad2.dpad_left){
                if(flyWheelPower >-1){
                    flyWheelPower-=.1;

                }
                while (gamepad2.dpad_left);
            }
            if(gamepad2.dpad_right){
                if(flyWheelPower < 1){
                    flyWheelPower += .1;
                }
                while(gamepad2.dpad_right);
            }
                telemetry.addData("Fly wheel 1 power: ", robot.flyWheel1.getPower());
                telemetry.addData("Fly wheel 2 power: ", robot.flyWheel2.getPower());
                telemetry.addData("Fly wheel 1 position: ", robot.flyWheel1.getCurrentPosition());
                telemetry.addData("Fly wheel 2 position: ", robot.flyWheel2.getCurrentPosition());

            if(gamepad2.x) {
                robot.flyWheel2.setPower(flyWheelPower);
                robot.flyWheel1.setPower(1);
                telemetry.addData("Fly Wheel 1 Speed", robot.flyWheel1.getPower());
                telemetry.addData("Fly Wheel 2 Speed", robot.flyWheel2.getPower());
            }

            if(!gamepad2.x){
                robot.flyWheel1.setPower(0);
                robot.flyWheel2.setPower(0);
            }

            if(gamepad2.right_stick_x < -0.8){
                if(gamepad2.x) {
                    robot.flyWheel1.setPower(-flyWheelPower);
                    robot.flyWheel2.setPower(-flyWheelPower);
                }
            }

            if (gamepad2.right_bumper){
                robot.extendotronRight.setPower(robot.EXTENDOTRON_LIFT_SPEED);
            }
            else if (gamepad2.right_trigger > 0){
                robot.extendotronRight.setPower(robot.EXTENDOTRON_DROP_SPEED);
            }
            else{
                robot.extendotronRight.setPower(0);
            }


            if(gamepad2.left_bumper){
                robot.extendotronLeft.setPower(robot.EXTENDOTRON_LIFT_SPEED);
            }
            else if(gamepad2.left_trigger > 0){
                robot.extendotronLeft.setPower(robot.EXTENDOTRON_DROP_SPEED);
            }
            else{
                robot.extendotronLeft.setPower(0);
            }

            //drive settings
            if (!driveDirection) {
                if (gamepad1.x) {
                    robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
                    robot.leftMotor.setDirection(DcMotor.Direction.FORWARD);
                    driveDirection = true;
                }
            }
            if (driveDirection) {
                if (gamepad1.x) {
                    robot.rightMotor.setDirection(DcMotor.Direction.FORWARD);
                    robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);
                    driveDirection = false;
                }
            }
            telemetry.update();

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
            lastTimeCalled = System.currentTimeMillis();
            lastPosition = motor.getCurrentPosition();
            long deltaPostion = motor.getCurrentPosition() - lastPosition;
            int ticksPerSecond = (int)(deltaPostion/(double)(deltaTime));
            int rps = (int)(ticksPerSecond/(double)(ticksPerRevolution));
            return rps;
        }

    }

}
