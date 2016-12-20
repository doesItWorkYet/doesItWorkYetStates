package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by root on 12/20/16.
 */
public class MotorController {
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

    public MotorController(DcMotor motor, double accelerationCoefficient, double startVelocity){
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
