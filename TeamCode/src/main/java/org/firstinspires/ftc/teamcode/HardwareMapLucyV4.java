package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.SensorManager;
import android.provider.Settings;

import com.qualcomm.robotcore.hardware.*;

/**
 * Created by root on 12/19/16.
 */
public class HardwareMapLucyV4 {
    //autonomous heading
    public double zeroedHeading;
    public double HEADING_ACCURACY = 8;
    public double[] baseLineColorAverage = {0,0,0};

    public final double ROBOT_WHEEL_RADIUS = .0476; //meters
    public final double ROBOT_WHEEL_TRACK = .244; //meters

    public final double ROBOT_WHEEL_CIRCUMFERENCE = ROBOT_WHEEL_RADIUS * 2 * Math.PI;

    // vars for Mat (special)
    public final double FOOT_TO_METERS = 12.0*2.54/100;

    //beacon color/side
    public final int BEACON_BLUE = 34563;
    public final int BEACON_RED = 121212;
    public final int BEACON_COLOR_UNKOWN = -1234;
    public final int BEACON_LEFT = 123;
    public final int BEACON_RIGHT = 345;

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
    public final double EXTENDORTON_LIFT_SPEED = .5;
    public Servo leftClawDeployer = null;
    public Servo rightClawDeployer = null;

    //T-Rex arms
    public Servo armletLeft = null;
    public Servo armletRight = null;

    //beanie cap propeller
    public DcMotor propeller = null;

    //beacon pressing buttons
    public Servo beaconPresserLeft = null;
    public Servo beaconPresserRight = null;


    final int BEACON_PRESSER_LEFT_STORE_POSITION = 70;
    final int BEACON_PRESSER_RIGHT_STORE_POSITION = 70;
    final int BEACON_PRESSER_LEFT_PRESS_POSITION = 100;
    final int BEACON_PRESSER_RIGHT_PRESS_POSITION = 90;


    final double BEACON_PRESSING_POWER = .5;//speed at which to ram wall

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

    final double WHITE_FUDGE_FACTOR = .2;
    final int COLOR_SENSOR_NUM_TIMES_CHECK_BACKGROUND_COLOR = 100;
    final int GROUND_COLOR_SENSOR_LED_PIN = 5;
    final int BEACON_COLOR_SENSOR_LED_PIN = 6;
    final int GROUND_COLOR_SENSOR_POWER_PIN = 4; //used to turn sensor on and off
    final int BEACON_COLOR_SENSOR_POWER_PIN = 3;
    final int BRIGHTNESS_WHITE_THREASHOLD = 4000;
    final int MIN_RED_VALUE_FOR_DETECTION = 80; //used to differentiate from background
    final int MIN_BLUE_VALUE_FOR_DETECTION = 80; //used to differentiate from background
    final double MIN_COLOR_MULTIPLIER = 1.8; // blue value must be x times larger than red
    final int USE_RGB = 1999;
    final int USE_BRIGHTNESS = 5050;

    final long BRAKE_TIME = 600;

    final double DISTANCE_INCREMEANT_FOR_WHITE_LINE_SEARCH = .02; //two centimeters
    final double ADVANCE_TO_WHITE_POWER = .6; //might as well as go fast....

    final double DEFAULT_POWER = 0.50;
    final double INCREASED_POWER = 0.53;


    //sensors
    private DeviceInterfaceModule sensorController = null;
    private ColorSensor rawGroundColorSensor = null;
    private ColorSensor rawBeaconColorSensor = null;
    private SensorManager manager;
    public Orientation orientation;
    public RGBSensor groundColorSensor = null;
    public RGBSensor beaconColorSensor = null;

    public TouchSensor leftBeaconPresserSensor = null;
    public TouchSensor rightBeaconPresserSensor = null;
    public UltrasonicSensor ultrasonicSensor;

    //beacon detection
    public BeaconDetector beaconDetection;
    public double RED_THREASHOLD = 1;
    public double BLUE_THREASHOLD = 1;

    HardwareMap hwMap = null;


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

            //prop init
            propeller = hwMap.dcMotor.get("prop");


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
            //armletRight.setDirection(Servo.Direction.REVERSE);

            //beacon pressing init
            beaconPresserLeft = hwMap.servo.get("leftBeaconPresser");
            beaconPresserRight = hwMap.servo.get("rightBeaconPresser");
            beaconPresserLeft.setDirection(Servo.Direction.FORWARD);
            beaconPresserRight.setDirection(Servo.Direction.REVERSE);

            sweep = hwMap.dcMotor.get("sweep");
            sweep.setDirection(DcMotor.Direction.REVERSE);

            //sensors
            sensorController = hwMap.deviceInterfaceModule.get("DIM");
            rawGroundColorSensor = hwMap.colorSensor.get("rawGroundColorSensor");
            rawBeaconColorSensor = hwMap.colorSensor.get("rawBeaconColorSensor");
            beaconColorSensor = new RGBSensor(rawBeaconColorSensor, sensorController, BEACON_COLOR_SENSOR_LED_PIN, true,BEACON_COLOR_SENSOR_POWER_PIN);
            groundColorSensor = new RGBSensor(rawGroundColorSensor, sensorController, GROUND_COLOR_SENSOR_LED_PIN, true,GROUND_COLOR_SENSOR_POWER_PIN);
            manager = (SensorManager) hwMap.appContext.getSystemService(Context.SENSOR_SERVICE);
            orientation = new Orientation(manager);



        }
        catch(Exception e){

        }

    }
    public void zero(){
        beaconPresserLeft.setPosition(BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
        beaconPresserRight.setPosition(BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
        indexer.setPosition(INDEXER_LOAD_POSITON/180.0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);
        sweep.setPower(0);
        armletLeft.setPosition(ARMLET_STORE_POSITION);
        armletRight.setPosition(ARMLET_STORE_POSITION);
        propeller.setPower(0);
    }

    public void turnToDegree(double degree){
        double distanceToTurn = (degree / 360.0) * (ROBOT_WHEEL_TRACK*Math.PI);
        double rotationsToTurn = distanceToTurn / (ROBOT_WHEEL_CIRCUMFERENCE);
        int ticksToTurn = (int) (rotationsToTurn * TICKS_PER_REV_ANDYMARK);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + ticksToTurn);
        rightMotor.setTargetPosition(rightMotor.getTargetPosition() - ticksToTurn);
        leftMotor.setPower(ROTATION_TURNING_SPEED);
        rightMotor.setPower(ROTATION_TURNING_SPEED);
        while(leftMotor.isBusy());
        while(rightMotor.isBusy());
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveDistance(double distance, double power) {
        double distanceInMeters = distance / 3.28084;
        double rotationsToTurn = distanceInMeters / ROBOT_WHEEL_CIRCUMFERENCE;
        int ticksToTurn = (int) (rotationsToTurn * TICKS_PER_REV_ANDYMARK);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + ticksToTurn);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + ticksToTurn);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        while(leftMotor.isBusy());
        while(rightMotor.isBusy());
        leftMotor.setPower(0); //release motors
        rightMotor.setPower(0); //release motors
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public double decideDirectionToTurn(double myHeading, double desiredHeading){
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


    public boolean checkIfWhite(double[] rgbValuesToCheck) {
        if (rgbValuesToCheck[0] > baseLineColorAverage[0] * (1 + WHITE_FUDGE_FACTOR)) {
            if (rgbValuesToCheck[1] > baseLineColorAverage[1] *(1 + WHITE_FUDGE_FACTOR)) {
                if (rgbValuesToCheck[2] > baseLineColorAverage[2] * (1 + WHITE_FUDGE_FACTOR)) {
                    return true;
                }
            }
        }
        return false;
    }

    public double[] getBaseLineColorState() {
        //assume we are using the ground sensor
        beaconColorSensor.turnSensorOff();
        groundColorSensor.waitForInitialization();
        double[] toReturn = {0,0,0};
        double[] rgbValues = groundColorSensor.getRGBColor();

        toReturn = rgbValues;
        for(int i = 0; i < COLOR_SENSOR_NUM_TIMES_CHECK_BACKGROUND_COLOR; i ++){
            rgbValues = groundColorSensor.getRGBColor();
            toReturn[0] = (toReturn[0] + rgbValues[0])/2.0;
            toReturn[1] = (toReturn[1] + rgbValues[1])/2.0;
            toReturn[2] = (toReturn[2] + rgbValues[2])/2.0;
        }
        return toReturn;
    }

    public void goForwardUntilWhite(int typeToUse) {
        //assume using ground sensor
        beaconColorSensor.turnSensorOff();
        groundColorSensor.waitForInitialization();
        if(typeToUse == USE_RGB) {
            baseLineColorAverage = getBaseLineColorState();
            groundColorSensor.turnLedOn();
            boolean runTimes = false;
            double[] newColor = groundColorSensor.getRGBColor();
            while (!runTimes) {
                if (!checkIfWhite(newColor)) {
                    driveDistance(DISTANCE_INCREMEANT_FOR_WHITE_LINE_SEARCH,ADVANCE_TO_WHITE_POWER);
                } else {
                    runTimes = true;
                }
            }
        }
        else if(typeToUse == USE_BRIGHTNESS){
            boolean hasCrossed = false;
            while(!hasCrossed){
                double brightness = groundColorSensor.getBrightness();
                if(brightness < BRIGHTNESS_WHITE_THREASHOLD){
                    driveDistance(DISTANCE_INCREMEANT_FOR_WHITE_LINE_SEARCH,ADVANCE_TO_WHITE_POWER);
                }
                else hasCrossed = true;
            }
        }
        brakeTemporarily(); //stop hard
    }

    public void delay(long time){
        long curTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < time + curTime);
    }

    public void brakeTemporarily(){
        //call this to have the robot stop fast
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delay(BRAKE_TIME);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void beaconApproach(int side) {
        if (side == BEACON_LEFT) {
            beaconPresserLeft.setPosition(ARMLET_DEPLOY_POSITION);
            while (!leftBeaconPresserSensor.isPressed()) {
                setDriveMotorPower(BEACON_PRESSING_POWER);
            }
            stop();

        }
        if (side == BEACON_RIGHT) {
            beaconPresserRight.setPosition(ARMLET_DEPLOY_POSITION);
            while (!rightBeaconPresserSensor.isPressed()) {
                setDriveMotorPower(BEACON_PRESSING_POWER);
            }

            stop();

        }
    }
    public void deployBeaconPressers(){
        beaconPresserLeft.setPosition(BEACON_PRESSER_LEFT_PRESS_POSITION);
        beaconPresserRight.setPosition(BEACON_PRESSER_RIGHT_PRESS_POSITION);
    }

}
