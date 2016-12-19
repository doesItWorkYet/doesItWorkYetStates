package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by root on 12/19/16.
 */
public class HardwareLucyV2 {
    //drive motors
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    //shooter motors
    public DcMotor flyWheel1 = null;
    public DcMotor flyWheel2 = null;
    public Servo indexer = null;

    //ball sweeper motors
    public DcMotor sweep = null;

    //cap ball lifter motors
    public DcMotor extendotronLeft = null;
    public DcMotor extendotronRight = null;
    public Servo leftClawDeployer = null;
    public Servo rightClawDeployer = null;


    //beacon pressing buttons
    public Servo beaconPresserLeft = null;
    public Servo beaconPresserRight = null;


    final int BEACON_PRESSER_STORE_POSITION = 0;
    final int BEACON_PRESSER_PRESS_POSITION = 90;

    final int INDEXER_LOAD_POSITON = 0;
    final int INDEXER_FIRE_POSITION = 90;

    final int TICKS_PER_REV_ANDYMARK = 1120;
    final int TICKS_PER_REV_TETRIX = 1440;

    final double ACCELERATION_OF_MAIN_MOTORS = .1;
    final double ACCELERATION_OF_FLY_WHEEL = .25;
    final double ACCELERATION_EXPONENT = 2;
    final double ACCELERATION_COEFFICIENT = .2;
    final double START_VELOCITY = .1;
    final double MAIN_MOTOR_MAX_POWER = .5;


    Sensor colorSensor = null;


    HardwareMap hwMap = null;


    public HardwareLucyV2(){

    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        //movement init
        leftMotor = hwMap.dcMotor.get("leftMotor");
        rightMotor = hwMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //flywheel init
        flyWheel1 = hwMap.dcMotor.get("flyWheel1");
        flyWheel2 = hwMap.dcMotor.get("flyWheel2");
        indexer = hwMap.servo.get("indexer");
        flyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


        //extendotron init
        //extendotronLeft = hwMap.dcMotor.get("");
        //extendotronRight = hwMap.dcMotor.get("");
        //leftClawDeployer = hwMap.servo.get("");
        //rightClawDeployer = hwMap.servo.get("");
        //extendotronLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //extendotronRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //beacon pressing init
        beaconPresserLeft = hwMap.servo.get("leftBeaconPresser");
        beaconPresserRight = hwMap.servo.get("rightBeaconPresser");
        beaconPresserLeft.setDirection(Servo.Direction.FORWARD);
        beaconPresserRight.setDirection(Servo.Direction.REVERSE);

        sweep = hwMap.dcMotor.get("sweep");
    }
    public void zero(){
        beaconPresserLeft.setPosition(BEACON_PRESSER_STORE_POSITION/180.0);
        beaconPresserRight.setPosition(BEACON_PRESSER_STORE_POSITION/180.0);
        indexer.setPosition(INDEXER_LOAD_POSITON/180.0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);
        sweep.setPower(0);
    }
}
