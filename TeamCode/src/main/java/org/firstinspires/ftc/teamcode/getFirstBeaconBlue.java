package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;


@Autonomous(name="Get First Beacon Blue", group="Testing")  // @Autonomous(...) is the other common choice

public class getFirstBeaconBlue extends LinearVisionOpMode {
    private double redTolerance = 0;
    private double blueTolerance = 0;

    HardwareMapLucyV4 robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero(this);
        waitOneFullHardwareCycle();
        telemetry.addData("Vars", "Set");
        telemetry.update();
        try {
            waitForVisionStart();
        } catch (Exception e) {
            telemetry.addData("Exception: ", e.getMessage());
        }


        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);
        telemetry.addData("Cameras", "Set");
        telemetry.update();
        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));
        telemetry.addData("Frame", "set");
        telemetry.update();
        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        boolean safety = robot.leftBeaconPresserSensor.isPressed() || robot.rightBeaconPresserSensor.isPressed();
        //Wait for start and reset the runtime count
        waitForStart();
        long timeToStop = System.currentTimeMillis() + 30*1000;
        robot.deployBeaconPressers();
        //while(opModeIsActive()){
        robot.driveDistance(robot.FEET_TO_TRAVEL_FROM_WALL, .25);
        robot.brakeTemporarily();
        robot.oneWheelTurn(robot.LEFT_MOTOR, robot.BEGIN_TURN_DEGREE_TO_GO_TO_WHITE_LINE_FROM_WALL, robot.TURNING_RPS);

        long currentPos = robot.leftMotor.getCurrentPosition();
        long ticksToTravelFast = robot.DIST_TO_TRAVEL_FAST_ON_WHITE_LINE_APPROACH*robot.TICKS_PER_REV_ANDYMARK;
        robot.beginSynchronousDriving(robot.FAST_RPS);
        while(robot.leftMotor.getCurrentPosition() < ticksToTravelFast + currentPos && opModeIsActive() && !safety){
            safety = robot.leftBeaconPresserSensor.isPressed() || robot.rightBeaconPresserSensor.isPressed();
            waitOneFullHardwareCycle();
        }
        robot.beginSynchronousDriving(robot.SLOW_RPS);
        while(robot.groundColorSensor.getBrightness()<robot.BRIGHTNESS_WHITE_THREASHOLD && !safety && opModeIsActive()) {
            safety = robot.leftBeaconPresserSensor.isPressed() || robot.rightBeaconPresserSensor.isPressed();
            waitOneFullHardwareCycle();
        }
        robot.endSynchronousDriving();

        if(robot.DEBUG){
            while(!robot.leftBeaconPresserSensor.isPressed() && !robot.rightBeaconPresserSensor.isPressed() && opModeIsActive()){
                waitOneFullHardwareCycle();
            }

        }

        if(!safety && opModeIsActive()) {
            robot.oneWheelTurn(robot.RIGHT_MOTOR, (90 - robot.BEGIN_TURN_DEGREE_TO_GO_TO_WHITE_LINE_FROM_WALL), robot.TURNING_RPS);
            //drive into wall
            robot.beginSynchronousDriving(robot.SLOW_RPS);
            while(robot.distSensor.getLightDetected() < robot.OPTICAL_SENSOR_THRESHOLD && opModeIsActive()){
                telemetry.addData("Light:", robot.distSensor.getLightDetected());
                telemetry.update();
                waitOneFullHardwareCycle();
            }

            robot.endSynchronousDriving();
            if(opModeIsActive()){
            //robot.driveStraightUntilWall(.15, robot.OPTICAL_SENSOR_THRESHOLD,timeToStop);
            telemetry.addData("Target:", "Within Range");
            telemetry.update();
            robot.brakeTemporarily();
            if(getLeftColor() == robot.BEACON_BLUE) {
                robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
                robot.delay(500);
                long start = System.currentTimeMillis();
                while(!robot.leftBeaconPresserSensor.isPressed() && opModeIsActive() && System.currentTimeMillis() < start + 2000){
                    robot.setDriveMotorPower(.3);
                    waitOneFullHardwareCycle();
                }
                waitOneFullHardwareCycle();
                robot.brakeTemporarily();
                telemetry.addData("Red on:", "Left");
                telemetry.update();
            }
            robot.brakeTemporarily();
            if(getRightColor() == robot.BEACON_BLUE) {

                robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION / 180.0);
                robot.delay(500);
                long start = System.currentTimeMillis();
                while (!robot.rightBeaconPresserSensor.isPressed() && opModeIsActive()&& System.currentTimeMillis() < start + 2000) {
                    robot.setDriveMotorPower(.3);
                    waitOneFullHardwareCycle();
                }
                waitOneFullHardwareCycle();
                robot.brakeTemporarily();
                telemetry.addData("Red on:", "Right");
                telemetry.update();
            }
            }
        }
        //turn to go to next line
        if(robot.DEBUG){
            while(!robot.leftBeaconPresserSensor.isPressed() && !robot.rightBeaconPresserSensor.isPressed() && opModeIsActive()){
                waitOneFullHardwareCycle();
            }
        }
        if(!safety && opModeIsActive()){
            //back up
            robot.driveDistance(-.2, .5);
/*
            //turn
            robot.oneWheelTurn(robot.LEFT_MOTOR,90, robot.TURNING_RPS);
            //go to next white line
            currentPos = robot.leftMotor.getCurrentPosition();
            ticksToTravelFast = robot.DIST_LINE_TO_LINE_FAST*robot.TICKS_PER_REV_ANDYMARK;
            robot.oneWheelTurn(robot.RIGHT_MOTOR, -robot.BEGIN_TURN_DEGREE_TO_GO_TO_WHITE_LINE_FROM_WALL, robot.TURNING_RPS);
            robot.beginSynchronousDriving(robot.FAST_RPS);
            telemetry.addData("Speed:", "Fast");
            telemetry.update();
            while(robot.leftMotor.getCurrentPosition() < ticksToTravelFast + currentPos && opModeIsActive() && safety){
                safety = robot.leftBeaconPresserSensor.isPressed() || robot.rightBeaconPresserSensor.isPressed();
                waitOneFullHardwareCycle();
            }
            telemetry.addData("Speed:", "Slow");
            telemetry.update();
            robot.beginSynchronousDriving(robot.SLOW_RPS);
            while(robot.groundColorSensor.getBrightness()<robot.BRIGHTNESS_WHITE_THREASHOLD && !safety && opModeIsActive()) {
                safety = robot.leftBeaconPresserSensor.isPressed() || robot.rightBeaconPresserSensor.isPressed();
                waitOneFullHardwareCycle();
                telemetry.addData("Brightness:", robot.groundColorSensor.getBrightness());
                telemetry.update();
            }
            telemetry.addData("Line:", "Detected");
            telemetry.update();
            //turn
        }
        robot.endSynchronousDriving();
        if(robot.DEBUG){
            while(!robot.leftBeaconPresserSensor.isPressed() && !robot.rightBeaconPresserSensor.isPressed() && opModeIsActive()){
                waitOneFullHardwareCycle();
            }
        }
        //beacon approach

        if(!safety && opModeIsActive()) {
            robot.oneWheelTurn(robot.LEFT_MOTOR, -90, robot.TURNING_RPS);
            robot.driveStraightUntilWall(.15, robot.OPTICAL_SENSOR_THRESHOLD,timeToStop);
            telemetry.addData("Target:", "Within Range");
            telemetry.update();
            robot.brakeTemporarily();
            if(getLeftColor() == robot.BEACON_RED) {
                robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
                //robot.delay(500);
                while(!robot.leftBeaconPresserSensor.isPressed() && opModeIsActive()){
                    robot.setDriveMotorPower(.3);
                    //robot.delay(150);
                    waitOneFullHardwareCycle();
                }
                if(opModeIsActive())
                    waitOneFullHardwareCycle();
                robot.brakeTemporarily();
                telemetry.addData("Red on:", "Left");
                telemetry.update();
            }
            robot.brakeTemporarily();
            if(getRightColor() == robot.BEACON_RED) {
                robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
                //robot.delay(500);
                while(!robot.rightBeaconPresserSensor.isPressed() && opModeIsActive()){
                    robot.setDriveMotorPower(.3);
                    waitOneFullHardwareCycle();
                }

                if(opModeIsActive())
                    waitOneFullHardwareCycle();
                robot.brakeTemporarily();
                telemetry.addData("Red on:", "Right");
                telemetry.update();
            }
*/
        }
    }
    private Beacon.BeaconAnalysis getAnalysis(){
        return beacon.getAnalysis();
    }

    public Point getBeaconCenter(){
        return getAnalysis().getCenter();
    }

    public Point[] getBeaconButtonLocations(){
        Point[] toReturn = {getAnalysis().getLeftButton().center(), getAnalysis().getRightButton().center()};
        return toReturn;
    }

    public boolean isBeaconCaptured(){
        if(getAnalysis().isLeftBlue() && getAnalysis().isRightBlue())  return true;
        if(getAnalysis().isLeftRed() && getAnalysis().isRightRed()) return true;
        return false;
    }

    public int getLeftColor(){
        if(getAnalysis().isLeftBlue()) return robot.BEACON_BLUE;
        if(getAnalysis().isLeftRed()) return robot.BEACON_RED;
        else return robot.BEACON_COLOR_UNKOWN;
    }


    public int getRightColor(){
        if(getAnalysis().isRightBlue()) return robot.BEACON_BLUE;
        if(getAnalysis().isRightRed()) return robot.BEACON_RED;
        else return robot.BEACON_COLOR_UNKOWN;
    }
}