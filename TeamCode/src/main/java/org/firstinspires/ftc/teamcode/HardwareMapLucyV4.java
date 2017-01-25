package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by root on 12/19/16.
 */
public class HardwareMapLucyV4 {
    public final boolean DEBUG = false;

    //autonomous heading
    public double zeroedHeading;
    public double HEADING_ACCURACY = 8;
    public double[] baseLineColorAverage = {0,0,0};

    public final int LEFT_MOTOR = 1;
    public final int RIGHT_MOTOR = -1;

    //Wheel Measurements
    private final double ROBOT_WHEEL_RADIUS = .047625; //meters
    public final double ROBOT_WHEEL_TRACK = .244; //meters
    private final double ROBOT_REV_PER_DEGREE = .013059301;
    private final double MARGIN_OF_ERROR_TURN = 15;
    public final double TURN_CORRECTION_FACTOR = 1;

    private final double GEAR_RATIO = 1;

    //Wheel Measurement Conversions
    public final double ROBOT_WHEEL_CIRCUMFERENCE = ROBOT_WHEEL_RADIUS * 2 * Math.PI;

    //Distance Measurements
    public final double CAP_BALL_DIST = 1.8;
    public final double SHOOTING_POSITION = 1;

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
    public double ROTATION_TURNING_SPEED = .5;
    public double COLOR_APPROACHING_SPEED = .2;

    //Ultrasonic Sensor Info
    public final int ULTRASONIC_SENSOR_READ_PIN = 4;
    public final int ULTRASONIC_SENSOR_TRIG_PIN = 5;


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
    public final double EXTENDOTRON_LIFT_SPEED = .75;
    public final double EXTENDOTRON_DROP_SPEED = -.25;
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

    //Beacon Presser Positions
    final int BEACON_PRESSER_LEFT_STORE_POSITION = 60;
    final int BEACON_PRESSER_RIGHT_STORE_POSITION = 60;
    final int BEACON_PRESSER_LEFT_PRESS_POSITION = 100;
    final int BEACON_PRESSER_RIGHT_PRESS_POSITION = 100;

    //Beacon Wall Approach Speed
    final double BEACON_PRESSING_POWER = .5;

    //Indexer Positions
    final double INDEXER_LOAD_POSITION = 0.0;
    final double INDEXER_FIRE_POSITION = 45.0;

    //Motor Tick Values
    final int TICKS_PER_REV_ANDYMARK = 1120;
    final int TICKS_PER_REV_TETRIX = 1440;

    //Miscellaneous Values
    final double PROPELLER_ON = 1;
    final double MOTOR_OFF = 0;
    final double FLY_WHEEL_POWER = 1;
    final double ACCELERATION_OF_MAIN_MOTORS = .1;
    final double ACCELERATION_OF_FLY_WHEEL = .25;
    final double ACCELERATION_EXPONENT = 2;
    final double ACCELERATION_COEFFICIENT = .2;
    final double START_VELOCITY = .1;
    final double MAIN_MOTOR_MAX_POWER = .8;

    final double WHITE_FUDGE_FACTOR = .2;
    final int COLOR_SENSOR_NUM_TIMES_CHECK_BACKGROUND_COLOR = 100;
    final int GROUND_COLOR_SENSOR_LED_PIN = 2;
    //final int BEACON_COLOR_SENSOR_LED_PIN = 3;
    //final int GROUND_COLOR_SENSOR_POWER_PIN = 1; //used to turn sensor on and off
    //final int BEACON_COLOR_SENSOR_POWER_PIN = 5;
    final double BRIGHTNESS_WHITE_THRESHOLD = .5;
    final int MIN_RED_VALUE_FOR_DETECTION = 80; //used to differentiate from background
    final int MIN_BLUE_VALUE_FOR_DETECTION = 80; //used to differentiate from background
    final double MIN_COLOR_MULTIPLIER = 1.8; // blue value must be x times larger than red
    final int USE_RGB = 1999;
    final int USE_BRIGHTNESS = 5050;

    final long BRAKE_TIME = 600;

    final double DISTANCE_INCREMENT_FOR_WHITE_LINE_SEARCH = .02; //two centimeters
    final double ADVANCE_TO_WHITE_POWER = .6; //might as well as go fast....

    final double DEFAULT_POWER = 0.50;
    final double INCREASED_POWER = 0.53;


    //sensors
    public DeviceInterfaceModule dim = null;
    private ColorSensor rawGroundColorSensor = null;
    //private ColorSensor rawBeaconColorSensor = null;
    private SensorManager manager;
    public Orientation orientation;
    public RGBSensor groundColorSensor = null;
    public TCS34725_ColorSensor fastColorSensor = null;
    //public RGBSensor beaconColorSensor = null;

    public TouchSensor leftBeaconPresserSensor = null;
    public TouchSensor rightBeaconPresserSensor = null;
    public UltrasonicSensor ultrasonicSensor;

    //beacon detection

    public double RED_THREASHOLD = 1;
    public double BLUE_THREASHOLD = 1;
    public int CAMERA_FRAME_WIDTH = 600;
    public int CAMERA_FRAME_HEIGHT = 600;
    String BLUE = "Blue";
    String RED = "Red";
    HardwareMap hwMap = null;

    public final double OPTICAL_SENSOR_THRESHOLD = .04;

    final int ARMLET_STORE_POSITION =  20;
    final int ARMLET_DEPLOY_POSITION = 160;

    final int DIST_TO_TRAVEL_FAST_ON_WHITE_LINE_APPROACH = 2;
    final double FAST_RPS = 1;
    final double SLOW_RPS = .5;
    final double WHITE_LINE_TURN_SPEED = .3;
    final double WALL_APPROACH_SPEED = .3;

    //definitions for Autonomous
    final int BEGIN_TURN_DEGREE_TO_GO_TO_WHITE_LINE_FROM_WALL = 58;
    final double FEET_TO_TRAVEL_FROM_WALL = 1.5;
    final double TURNING_RPS = .3;
    final int DIST_LINE_TO_LINE_FAST = 1;

    public OpticalDistanceSensor distSensor = null;

    public HardwareMapLucyV4(){

    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        try {
            //movement init
            leftMotor = hwMap.dcMotor.get("leftMotor");
            rightMotor = hwMap.dcMotor.get("rightMotor");
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            extendotronLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            extendotronRight.setDirection(DcMotorSimple.Direction.FORWARD);

            //armlets
            armletLeft = hwMap.servo.get("armletLeft");
            armletRight = hwMap.servo.get("armletRight");
            armletLeft.setDirection(Servo.Direction.REVERSE);

            //beacon pressing init
            beaconPresserLeft = hwMap.servo.get("leftBeaconPresser");
            beaconPresserRight = hwMap.servo.get("rightBeaconPresser");
            beaconPresserLeft.setDirection(Servo.Direction.FORWARD);
            beaconPresserRight.setDirection(Servo.Direction.REVERSE);

            sweep = hwMap.dcMotor.get("sweep");
            sweep.setDirection(DcMotor.Direction.REVERSE);

            //sensors
            dim = hwMap.deviceInterfaceModule.get("DIM");

            distSensor = hwMap.opticalDistanceSensor.get("distSensor");


            //ultrasonicSensor = new UltrasonicSensor(ULTRASONIC_SENSOR_READ_PIN,ULTRASONIC_SENSOR_TRIG_PIN, dim);
            leftBeaconPresserSensor = hwMap.touchSensor.get("safetyLeft");
            rightBeaconPresserSensor = hwMap.touchSensor.get("safetyRight");


            //rawGroundColorSensor = hwMap.colorSensor.get("rawGroundColorSensor");
            //rawBeaconColorSensor = hwMap.colorSensor.get("rawBeaconColorSensor");
            //beaconColorSensor = new RGBSensor("rawBeaconColorSensor",hwMap, dim, BEACON_COLOR_SENSOR_LED_PIN, true,BEACON_COLOR_SENSOR_POWER_PIN);
            groundColorSensor = new RGBSensor("rawGroundColorSensor",hwMap, dim, GROUND_COLOR_SENSOR_LED_PIN, true);
            manager = (SensorManager) hwMap.appContext.getSystemService(Context.SENSOR_SERVICE);
            orientation = new Orientation(manager);
            fastColorSensor = new TCS34725_ColorSensor(hwMap, "fastColorSensor");




        }
        catch(Exception e){

        }

    }
    public void zero(OpMode mode) throws InterruptedException {
        beaconPresserLeft.setPosition(BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
        beaconPresserRight.setPosition(BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
        indexer.setPosition(INDEXER_LOAD_POSITION /180.0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);
        sweep.setPower(0);
        armletLeft.setPosition(ARMLET_STORE_POSITION/180.0);
        armletRight.setPosition(ARMLET_STORE_POSITION/180.0);
        propeller.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (mode instanceof LinearVisionOpMode) {
            ((LinearVisionOpMode) mode).waitOneFullHardwareCycle();
        }
        else if (mode instanceof LinearOpMode) {
            ((LinearOpMode) mode).idle();
        }
/*
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(leftMotor.getCurrentPosition() != 0);
        while(rightMotor.getCurrentPosition() != 0);
    */
    }

    public void driveDistance(double distance, double power, OpMode mode) {
        double distanceInMeters = distance / 3.28084;
        double rotationsToTurn = distanceInMeters / ROBOT_WHEEL_CIRCUMFERENCE;
        int ticksToTurn = (int) (rotationsToTurn * TICKS_PER_REV_ANDYMARK);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + ticksToTurn);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + ticksToTurn);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        while(leftMotor.isBusy() && !safety(mode));
        while(rightMotor.isBusy() && !safety(mode));
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(0); //release motors
        rightMotor.setPower(0); //release motors
    }

    public void driveStraightUntilWall(double speed, double opticalSensorThreshold, long timeToQuit, OpMode mode){
        double lightLevel = distSensor.getLightDetected();
        leftMotor.setMaxSpeed((int)(TICKS_PER_REV_ANDYMARK*speed));
        rightMotor.setMaxSpeed((int)(TICKS_PER_REV_ANDYMARK*speed));

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setDriveMotorPower(speed);
        while(distSensor.getLightDetected() < opticalSensorThreshold && System.currentTimeMillis() < timeToQuit && !safety(mode)){

        };
        stop();
        brakeTemporarily(mode);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void beginSynchronousDriving(double rps){
        leftMotor.setMaxSpeed((int)(TICKS_PER_REV_ANDYMARK*rps));
        rightMotor.setMaxSpeed((int)(TICKS_PER_REV_ANDYMARK*rps));

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setDriveMotorPower(rps);
    }

    public void endSynchronousDriving(OpMode mode){
        stop();
        brakeTemporarily(mode);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //stop
        //brake
        //reset encoders

    }


    /*
    public void driveAtSpeed(double distance, double speed) {
        double distanceInMeters = distance / 3.28084;
        double rotationsToTurn = distanceInMeters / ROBOT_WHEEL_CIRCUMFERENCE;
        int ticksToTurn = (int) (rotationsToTurn * TICKS_PER_REV_ANDYMARK);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMaxSpeed();
    }

*/

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

    public void oneWheelTurn(int motor, double degree, double power, OpMode mode){
        int direction = 0;
        if(degree>0){
            direction = (RIGHT_MOTOR == motor)? -1 : 1;
        }
        if(degree<0){
            direction = (RIGHT_MOTOR == motor)? 1 : -1;
        }
        DcMotor dcMotor = (motor == RIGHT_MOTOR)? rightMotor : leftMotor;
        double rotations = Math.abs(degree)*ROBOT_REV_PER_DEGREE;
        double ticks = rotations * TICKS_PER_REV_ANDYMARK * TURN_CORRECTION_FACTOR;
        int currentPosition = (dcMotor.getCurrentPosition());
        int targetPosition = (int) (currentPosition + (direction * ticks));
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotor.setTargetPosition(targetPosition);
        dcMotor.setPower(power);
        while(dcMotor.isBusy() && !safety(mode));
        dcMotor.setPower(0);
        dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brakeTemporarily(mode);
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
        //groundColorSensor.turnSensorOff();
        fastColorSensor.waitForInitialization();
        double[] toReturn = {0,0,0};
        double[] rgbValues = fastColorSensor.getRGBColor();

        toReturn = rgbValues;
        for(int i = 0; i < COLOR_SENSOR_NUM_TIMES_CHECK_BACKGROUND_COLOR; i ++){
            rgbValues = fastColorSensor.getRGBColor();
            toReturn[0] = (toReturn[0] + rgbValues[0])/2.0;
            toReturn[1] = (toReturn[1] + rgbValues[1])/2.0;
            toReturn[2] = (toReturn[2] + rgbValues[2])/2.0;
        }
        return toReturn;
    }

    public void goForwardUntilWhite(int typeToUse, OpMode mode) {
        //assume using ground sensor
        //.turnSensorOff();
        boolean safetyLeft = leftBeaconPresserSensor.isPressed();
        boolean safetyRight = rightBeaconPresserSensor.isPressed();
        fastColorSensor.waitForInitialization();
        if(typeToUse == USE_RGB) {
            baseLineColorAverage = getBaseLineColorState();
            boolean runTimes = false;
            double[] newColor = fastColorSensor.getRGBColor();
            while (!runTimes && !safetyLeft && !safetyRight && !safety(mode)) {
                if (!checkIfWhite(newColor)&& !safetyLeft && !safetyRight) {
                    driveDistance(DISTANCE_INCREMENT_FOR_WHITE_LINE_SEARCH,ADVANCE_TO_WHITE_POWER, mode);
                } else {
                    runTimes = true;
                }
                if(safetyLeft||safetyRight){
                    brakeTemporarily(mode);
                }
            }
        }

        else if(typeToUse == USE_BRIGHTNESS){
            boolean hasCrossed = false;
            while(!hasCrossed && !safetyLeft && !safetyRight){
                double brightness = fastColorSensor.getBrightness();
                if(brightness < BRIGHTNESS_WHITE_THRESHOLD && !safetyLeft && !safetyRight){
                    driveDistance(DISTANCE_INCREMENT_FOR_WHITE_LINE_SEARCH,ADVANCE_TO_WHITE_POWER, mode);
                }
                else hasCrossed = true;
                if(safetyRight||safetyLeft){
                    brakeTemporarily(mode);
                }
            }
        }
        brakeTemporarily(mode); //stop hard
    }


    public void delay(long time, OpMode mode){
        long curTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < time + curTime && !safety(mode));
    }

    public void brakeTemporarily(OpMode mode){
        //call this to have the robot stop fast
        stop();
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delay(BRAKE_TIME, mode);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void beaconApproach(int side, OpMode mode) {
        if (side == BEACON_LEFT) {
            beaconPresserLeft.setPosition(ARMLET_DEPLOY_POSITION);
            while (!leftBeaconPresserSensor.isPressed() && !safety(mode)) {
                setDriveMotorPower(BEACON_PRESSING_POWER);
            }
            stop();

        }
        if (side == BEACON_RIGHT) {
            beaconPresserRight.setPosition(ARMLET_DEPLOY_POSITION);
            while (!rightBeaconPresserSensor.isPressed() && !safety(mode)) {
                setDriveMotorPower(BEACON_PRESSING_POWER);
            }

            stop();

        }
    }
    public void deployBeaconPressers(){
        beaconPresserLeft.setPosition(BEACON_PRESSER_LEFT_PRESS_POSITION);
        beaconPresserRight.setPosition(BEACON_PRESSER_RIGHT_PRESS_POSITION);
    }

    public void deployArmlets(){
        armletRight.setPosition(ARMLET_DEPLOY_POSITION);
        armletLeft.setPosition(ARMLET_DEPLOY_POSITION);
    }

    public void storeArmlets(){
        armletLeft.setPosition(ARMLET_STORE_POSITION);
        armletRight.setPosition(ARMLET_STORE_POSITION);
    }

    public boolean safety(OpMode mode){
        boolean isOpModeActive;
        boolean wallIsNear = false;
        if (mode instanceof LinearVisionOpMode) {
            isOpModeActive = true;
        }
        else if (mode instanceof LinearOpMode) {
            isOpModeActive = true;
        }
        else {
            isOpModeActive = false;
        }
        if(distSensor.getLightDetected() < OPTICAL_SENSOR_THRESHOLD){
            wallIsNear = true;
        }
        if (!wallIsNear && isOpModeActive) {
            return false;
        }
        else { return true; }
    }

    public void shoot(OpMode mode){
        flyWheel2.setPower(FLY_WHEEL_POWER);
        flyWheel1.setPower(FLY_WHEEL_POWER);
        // to be completed
    }

    /*
    public void stayStraight(int desiredHeading, OpMode mode){
        boolean initialDirection = false; //false means turn left, true is turn right
        if (Math.abs(0 - desiredHeading) > Math.abs(359-desiredHeading)) {
            initialDirection = false;
        }
        if (Math.abs(0 - desiredHeading) <= Math.abs(359-desiredHeading)) {
            initialDirection = true;
        }
        if (initialDirection) {
            while (Math.abs(currentHeading - desiredHeading) > 5 && !safety(mode)) {
                oneWheelTurn(LEFT_MOTOR, 0.5, 1, mode);
            }
        }
        if (!initialDirection) {
            while (Math.abs(currentHeading - desiredHeading) > 5 && !safety(mode)) {
                oneWheelTurn(RIGHT_MOTOR, 0.5, 1, mode);
            }
        }
        double brightness = fastColorSensor.getBrightness();
        while (brightness < BRIGHTNESS_WHITE_THRESHOLD && !safety(mode)) {
            while (currentHeading == desiredHeading && !safety(mode)) {
                driveDistance(DISTANCE_INCREMENT_FOR_WHITE_LINE_SEARCH, 0.9, mode);
            }
            if (currentHeading < desiredHeading) {
                leftMotor.setPower(1);
                rightMotor.setPower(0.9);
            }
            if (currentHeading > desiredHeading) {
                rightMotor.setPower(1);
                leftMotor.setPower(0.9);
            }
        }
        stop();

    }
*/
}
