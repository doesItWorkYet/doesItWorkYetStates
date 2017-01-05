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
public class lucyV4HardwareMap extends LinearOpMode {
    HardwareMapLucyV4 robot;
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
        robot.zero();
        telemetry.addData("Status", "Robot zeros");
        telemetry.update();
        driveLeftController = new motorController(robot.leftMotor,robot.ACCELERATION_COEFFICIENT,robot.START_VELOCITY);
        driveRightController = new motorController(robot.rightMotor,robot.ACCELERATION_COEFFICIENT,robot.START_VELOCITY);
        RPMCounter flyWheel1Counter = new RPMCounter(robot.flyWheel1, robot.TICKS_PER_REV_ANDYMARK);
        RPMCounter flyWheel2Counter = new RPMCounter(robot.flyWheel2, robot.TICKS_PER_REV_ANDYMARK);
        waitForStart();
        runtime.reset();

        //Declare variables for use in runtime loop
        int rps1 = 1;
        int rps2 = 1;
        boolean reverse = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("flyWheel1", "RPS: "+ flyWheel1Counter.getRPS());
            telemetry.addData("flyWheel2", "RPS: "+ flyWheel2Counter.getRPS());
            telemetry.update();
            //When the right trigger is fully pressed, turn on the flywheel motors
            if(gamepad2.x){
                robot.flyWheel1.setPower(rps1);
                robot.flyWheel2.setPower(rps2);
            }

            //If the right trigger is not fully pressed, ensure the flywheel motors are turned off
            if(!gamepad2.x){
                robot.flyWheel1.setPower(0);
                robot.flyWheel2.setPower(0);
            }

            //Move the indexer into the load position
            if(gamepad2.y){
                robot.indexer.setPosition(80.0/360.0);
            }
            //Move the indexer back to default position
            else if(!gamepad2.y) {
                robot.indexer.setPosition(0.0);
            }

            if(gamepad2.dpad_up){
                robot.armletRight.setPosition(robot.ARMLET_DEPLOY_POSITION);
                robot.armletLeft.setPosition(robot.ARMLET_DEPLOY_POSITION);
            }

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
            if(gamepad2.a) {
                robot.sweep.setPower(1);
            }
            //When the B button is pressed, turn the sweep motor in the negative direction
            if(gamepad2.b) {
                robot.sweep.setPower(-1);
            }
            //If neither the A or B button are pressed, turn off the sweep motor
            if(!gamepad2.a & !gamepad2.b) {
                robot.sweep.setPower(0);
            }

            if(gamepad2.dpad_right){
                robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_PRESS_POSITION);
                robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_PRESS_POSITION);
            }
            if(gamepad2.dpad_left){
                robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION);
                robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION);
            }

            //cap ball lift controls:
            /*NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
            if(gamepad2.right_bumper){
                robot.extendotronRight.setPower(1);
            }
            else{
                robot.extendotronRight.setPower(0);
            }
            if(gamepad2.right_trigger>0){
                robot.extendotronRight.setPower(-1);
            }
            else{
                robot.extendotronRight.setPower(0);
            }
            if(gamepad2.left_bumper){
                robot.extendotronLeft.setPower(1);
            }
            else{
                robot.extendotronLeft.setPower(0);
            }
            if(gamepad2.left_trigger>0){
                robot.extendotronLeft.setPower(-1);
            }
            else{
                robot.extendotronLeft.setPower(0);
            }
            */
            if(gamepad2.right_bumper){
                robot.extendotronRight.setPower(robot.EXTENDORTON_LIFT_SPEED);
            }
            else if(gamepad2.right_trigger > 0){
                robot.extendotronRight.setPower(-robot.EXTENDORTON_LIFT_SPEED);
            }
            else{
                robot.extendotronRight.setPower(0);
            }


            if(gamepad2.left_bumper){
                robot.extendotronLeft.setPower(robot.EXTENDORTON_LIFT_SPEED);
            }
            else if(gamepad2.left_trigger > 0){
                robot.extendotronLeft.setPower(-robot.EXTENDORTON_LIFT_SPEED);
            }
            else{
                robot.extendotronLeft.setPower(0);
            }

            //drive settings
            if(gamepad1.a){
                robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
                robot.leftMotor.setDirection(DcMotor.Direction.FORWARD);
            }
            else if(gamepad1.b){
                robot.rightMotor.setDirection(DcMotor.Direction.FORWARD);
                robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);
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
            lastTimeCalled = System.currentTimeMillis();
            lastPosition = motor.getCurrentPosition();
            long deltaPostion = motor.getCurrentPosition() - lastPosition;
            int ticksPerSecond = (int)(deltaPostion/(double)(deltaTime));
            int rps = (int)(ticksPerSecond/(double)(ticksPerRevolution));
            return rps;
        }
    }

}
