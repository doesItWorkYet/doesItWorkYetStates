package org.firstinspires.ftc.teamcode;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

public class getFirstBeaconRedProportionalTest extends LinearVisionOpMode {
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

        // code starts
        robot.deployBeaconPressers();
        // 1. Drive using gyro sensor from 0° to 90° for a distance of 5.5’
        robot.driveDistanceFollowingHeadingProportional(-90, 0.8, 0.6, 5.5, this);
        // 2. Shoot two particles into center vortex in passing
        // TODO - add shooting of particles here
        // 3. Follow white line until proximity sensor detects beacon
        // find white line here
        robot.followLineStraightRed(0.35, 0.2, this);
        // 4. Select beacon presser according to color and punch it
        robot.selectBeaconColor(getLeftColor(), robot.BEACON_RED);
        robot.pressBeacon(0.3, 600, this);

        // round two
        robot.deployBeaconPressers();
        // 5. Back up to a heading of 0°
        robot.driveToHeadingProportional(0, -0.8, -0.6, this);
        // 6. Drive straight for 3', then continue to white line
        robot.driveDistanceFollowingHeadingProportional(0, 0.8, 0.6, 3.0, this);
        robot.beginSynchronousDriving(1, 0.4);
        while(robot.fastColorSensor.getBrightness()<robot.BRIGHTNESS_WHITE_THRESHOLD){
            telemetry.addData("Brightness: ", robot.fastColorSensor.getBrightness());
            telemetry.update();
        }
        robot.endSynchronousDriving(this);
        // 7. turn to white line and follow white line until proximity sensor detects beacon
        robot.turnToHeadingProportionalControl(robot.LEFT_MOTOR, -45, -0.6, -0.4, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        robot.turnToHeadingProportionalControl(robot.RIGHT_MOTOR, -90, 0.6, 0.4, robot.TURNING_P, robot.HEADING_ACCURACY, this);
        robot.followLineStraightRed(0.35, 0.2, this);
        // 8. Select beacon presser according to color and punch it
        robot.selectBeaconColor(getLeftColor(), robot.BEACON_RED);
        robot.pressBeacon(0.3, 600, this);
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