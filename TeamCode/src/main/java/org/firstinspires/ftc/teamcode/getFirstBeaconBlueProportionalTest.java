package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

@Autonomous(name="Get Beacon Proportional Test Blue", group = "Testing")
public class getFirstBeaconBlueProportionalTest extends LinearVisionOpMode {
    private double redTolerance = 0;
    private double blueTolerance = 0;

    HardwareMapLucyV4 robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero(this);
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
        this.setCamera(Cameras.SECONDARY);
        telemetry.addData("Cameras", "Set");
        telemetry.update();
        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(600, 600));
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

        rotation.setIsUsingSecondaryCamera(true);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        waitForStart();
        robot.calibrateGyro(this);
        robot.deployBeaconPressers();
        telemetry.addData("Start", "");
        telemetry.update();
        robot.driveDistanceFollowingHeadingProportional(33, 0.8, 0.5, 4, this);
        robot.followingHeadingToWhiteLine(33, 0.3, 0.2, this);
        telemetry.addData("Find Line", "");
        telemetry.update();
        robot.brakeTemporarily(this);
        telemetry.addData("Start turn", "");
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.LEFT_MOTOR, 60, 0.3, 0.2, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        telemetry.addData("turn two start", "");
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.RIGHT_MOTOR, 80, 0.3, 0.2, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        telemetry.addData("End Turn", "");
        telemetry.update();
        // 2. Shoot two particles into center vortex in passing
        // TODO - add shooting of particles here
        // 3. Follow white line until proximity sensor detects beacon
        // find white line here
//        robot.followLineStraightRed(0.15, 0.08, this);
        telemetry.addData("Go to wall", "");
        telemetry.update();
        robot.driveStraightUntilWall(0.45, robot.OPTICAL_SENSOR_THRESHOLD, this);
        telemetry.addData("Stop at wall", "");
        telemetry.update();
        // 4. Select beacon presser according to color and punch it
        robot.selectBeaconColor(getLeftColor(), robot.BEACON_BLUE, this);
        telemetry.addData("Color Selected", "");
        telemetry.update();
        robot.delay(500, this, true);
        robot.pressBeacon(0.3, 600, this);
        telemetry.addData("Got beacon", "");
        telemetry.update();


        // round two
        robot.deployBeaconPressers();
        telemetry.addData("Start round two", "");
        telemetry.update();
        // 5. Back up to a heading of 0°
        telemetry.addData("Back up", "");
        telemetry.update();
        robot.driveDistanceFollowingHeading(80, -0.8, -0.7, -0.5, this);
        telemetry.addData("turn", "");
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.LEFT_MOTOR, 0, 0.4, 0.3, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        telemetry.addData("Go over white line", "");
        telemetry.update();
        // 6. Drive straight for 3', then continue to white line
        robot.driveDistanceFollowingHeadingProportional(0, 0.8, 0.6, 3.0, this);
        telemetry.addData("Go to white line", "");
        telemetry.update();
        robot.beginSynchronousDriving(1, 0.4);
        while(robot.fastColorSensor.getBrightness()<robot.BRIGHTNESS_WHITE_THRESHOLD){
            telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
            telemetry.update();
        }
        robot.endSynchronousDriving(this);
        telemetry.addData("found line", "");
//        telemetry.update();
//        robot.driveDistanceFollowingHeading(0, 0.5, 0.4, 0.08, this);
//        robot.brakeTemporarily(this);
//        telemetry.addData("go further", "");
        // 7. turn to white line and follow white line until proximity sensor detects beacon
        telemetry.addData("turn 1", "");
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.LEFT_MOTOR, 45, 0.6, 0.4, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        telemetry.addData("turn 2", "");
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.RIGHT_MOTOR, 75, 0.6, 0.4, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        telemetry.addData("go to wall", "");
        telemetry.update();
        robot.driveStraightUntilWall(0.5, robot.OPTICAL_SENSOR_THRESHOLD-0.005, this);
        telemetry.addData("at wall", "");
        telemetry.addData("select color", "");
        telemetry.update();
        // 8. Select beacon presser according to color and punch it
        robot.selectBeaconColor(getLeftColor(), robot.BEACON_RED, this);
        telemetry.addData("get beacon", "");
        telemetry.update();
        robot.delay(450, this, true);
        robot.pressBeacon(0.3, 600, this);
        telemetry.addData("done", "");
        telemetry.update();
        waitOneFullHardwareCycle();
        while(opModeIsActive()){
            telemetry.addData("Heading: ", robot.gyro.getIntegratedZValue());
            telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
            telemetry.update();
            waitOneFullHardwareCycle();
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