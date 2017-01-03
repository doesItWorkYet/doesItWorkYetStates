package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.hardware.*;

/**
 * Created by root on 12/19/16.
 */
public class HardwareMapLucyV4 {
    //autonomous heading
    public double zeroedHeading;
    public double HEADING_ACCURACY = 8;

    public final double ROBOT_WHEEL_RADIUS = .0505; //meters
    public final double ROBOT_WHEEL_TRACK = .244; //meters

    public final double ROBOT_WHEEL_CIRCUMFERENCE = ROBOT_WHEEL_RADIUS * 2 * Math.PI;

    //beacon pressing autonomous
    public double BEACON_HITTING_ROTATION = -90;
    public double ADVANCE_TO_BEACON_HEADING = -30;
    public double ROTATION_TURNING_SPEED = .1;
    public double COLOR_APPROACHING_SPEED = .2;

    public final int ULTRASONIC_SENSOR_READ_PIN = 3;
    public final int ULTRASONIC_SENSOR_TRIG_PIN = 4;


    //drive motors
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public MotorController leftMotorController = null;
    public MotorController rightMotorController = null;

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

    //T-Rex arms
    public Servo armletLeft = null;
    public Servo armletRight = null;

    //beacon pressing buttons
    public Servo beaconPresserLeft = null;
    public Servo beaconPresserRight = null;


    final int BEACON_PRESSER_STORE_POSITION = 60;
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
    final double MAIN_MOTOR_MAX_POWER = .8;
    final double WHITE_FUDGE_FACTOR = .1;
    final int COLOR_SENSOR_NUM_TIMES_CHECK_BACKGROUND_COLOR = 100;
    final double DEFAULT_POWER = 0.50;
    final double INCREASED_POWER = 0.53;

    //sensors
    private DeviceInterfaceModule sensorController = null;
    private ColorSensor rawColorSensor = null;
    private SensorManager manager;
    public Orientation orientation;
    public RGBSensor colorSensor;
    public TouchSensor leftBeaconPresserSensor = null;
    public TouchSensor rightBeaconPresserSensor = null;
    public UltrasonicSensor ultrasonicSensor;

    //beacon detection
    public BeaconDetector beaconDetection;
    public double RED_THREASHOLD = 1;
    public double BLUE_THREASHOLD = 1;

    HardwareMap hwMap = null;
    final int COLOR_SENSOR_LED_PIN = 5;


    final int ARMLET_STORE_POSITION =  50;
    final int ARMLET_DEPLOY_POSITION = 140;

    public final int BLUE = 1;
    public final int RED = -1;
    public final int UNKOWN_COLOR = 0;

    public HardwareMapLucyV4(){

    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        try {
            //movement init
            leftMotor = hwMap.dcMotor.get("leftMotor");
            rightMotor = hwMap.dcMotor.get("rightMotor");
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftMotorController = new MotorController(leftMotor, ACCELERATION_COEFFICIENT, START_VELOCITY);
            rightMotorController = new MotorController(rightMotor, ACCELERATION_COEFFICIENT, START_VELOCITY);

            //flywheel init
            flyWheel1 = hwMap.dcMotor.get("flyWheel1");
            flyWheel2 = hwMap.dcMotor.get("flyWheel2");
            indexer = hwMap.servo.get("indexer");
            flyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
            flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);


            //extendotron init
            extendotronLeft = hwMap.dcMotor.get("liftLeft");
            extendotronRight = hwMap.dcMotor.get("liftRight");
            //leftClawDeployer = hwMap.servo.get("");
            //rightClawDeployer = hwMap.servo.get("");
            extendotronLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            extendotronRight.setDirection(DcMotorSimple.Direction.FORWARD);

            //armlets
            armletLeft = hwMap.servo.get("armletLeft");
            armletRight = hwMap.servo.get("armletRight");
            armletRight.setDirection(Servo.Direction.REVERSE);

            //beacon pressing init
            beaconPresserLeft = hwMap.servo.get("leftBeaconPresser");
            beaconPresserRight = hwMap.servo.get("rightBeaconPresser");
            beaconPresserLeft.setDirection(Servo.Direction.FORWARD);
            beaconPresserRight.setDirection(Servo.Direction.REVERSE);

            sweep = hwMap.dcMotor.get("sweep");
            sweep.setDirection(DcMotor.Direction.REVERSE);

            //sensors
            sensorController = hwMap.deviceInterfaceModule.get("DIM");
            rawColorSensor = hwMap.colorSensor.get("rawColorSensor");
            manager = (SensorManager) hwMap.appContext.getSystemService(Context.SENSOR_SERVICE);
            orientation = new Orientation(manager);
            colorSensor = new RGBSensor(rawColorSensor, sensorController, COLOR_SENSOR_LED_PIN, true);
            ultrasonicSensor = new UltrasonicSensor(ULTRASONIC_SENSOR_READ_PIN,ULTRASONIC_SENSOR_TRIG_PIN, sensorController);

            //extend linearVisionOpMode for autonomous
            //beacon detection
            //beaconDetection = new BeaconDetector();
            //beaconDetection.beginDetection(RED_THREASHOLD, BLUE_THREASHOLD);
        }
        catch(Exception e){

        }

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
        armletLeft.setPosition(ARMLET_STORE_POSITION);
        armletRight.setPosition(ARMLET_STORE_POSITION);
    }

    public void turnToDegree(double degree){
        double distanceToTurn = (degree / 360.0) * (ROBOT_WHEEL_TRACK*Math.PI);
        double rotationsToTurn = distanceToTurn / (ROBOT_WHEEL_CIRCUMFERENCE);

    }

    public double degreeToRadian(int degree){
        return degree/180.0;
    }

    public double radianToDegree(double radians){
        return radians*180;
    }

    public void turn(double power){
        rightMotor.setPower(-power);
        leftMotor.setPower(power);
    }

    public void stop(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void setDriveMotorPower(double power){
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    public void setZeroHeading(double heading){
        zeroedHeading = heading;
    }

    public double convertToRelativeHeading(double heading){
        double toReturn = heading - zeroedHeading;
        if(toReturn < 0) toReturn += 360;
        if(toReturn > 360) toReturn -= 360;
        return toReturn;
    }

    public double decideDriectionToTurn(double myHeading, double desiredHeading){
        //get the angle deficit to left and right
        double rightDeficit, leftDeficit;
        if(myHeading < desiredHeading){
            rightDeficit = desiredHeading - myHeading;
            leftDeficit = rightDeficit - 360;
        }
        else{
            rightDeficit = 360 - Math.abs(desiredHeading - myHeading);
            leftDeficit =  rightDeficit - 360;
        }
        if(Math.abs(rightDeficit) < Math.abs(leftDeficit)) return 1;
        else return -1;
    }

}
