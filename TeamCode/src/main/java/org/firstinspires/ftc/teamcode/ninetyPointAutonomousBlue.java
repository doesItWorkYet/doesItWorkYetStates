package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;


@Autonomous(name="90 point autonomous", group="Testing")  // @Autonomous(...) is the other common choice

public class ninetyPointAutonomousBlue extends LinearVisionOpMode {
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
        this.setCamera(Cameras.SECONDARY);
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

        rotation.setIsUsingSecondaryCamera(true);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        telemetry.addData("approach line", "");
        telemetry.update();
        robot.driveDistance(robot.DIST_TO_TRAVEL_FAST_ON_WHITE_LINE_APPROACH, 0.5, this);
        telemetry.addData("go to line", "");
        telemetry.update();
        robot.beginSynchronousDriving(robot.FAST_RPS, 0.5);
        while(!robot.safety(this) && robot.fastColorSensor.getBrightness()<robot.BRIGHTNESS_WHITE_THRESHOLD);
        robot.endSynchronousDriving(this);
        telemetry.addData("turn to beacon", "");
        telemetry.update();
        robot.oneWheelTurn(robot.LEFT_MOTOR, 58, 0.25, this);
        telemetry.addData("follow line", "");
        telemetry.update();
        robot.followLineStraightBlue(0.35, 0.1, this);
        telemetry.addData("get beacon 1", "");
        telemetry.update();
        if(getLeftColor() == robot.BEACON_BLUE){
            robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
        }
        else{
            robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
        }
        robot.driveDistance(0.15, 0.3, this);
        robot.driveDistance(-0.15, 0.4, this);
        telemetry.addData("turn to other line", "");
        telemetry.update();
        robot.oneWheelTurn(robot.LEFT_MOTOR, -90, 0.5, this);
        telemetry.addData("approach line 2", "");
        telemetry.update();
        robot.driveDistance(robot.DIST_TO_TRAVEL_FAST_ON_WHITE_LINE_APPROACH, 0.5, this);
        telemetry.addData("go to line", "");
        telemetry.update();
        robot.beginSynchronousDriving(robot.FAST_RPS, 0.5);
        while(!robot.safety(this) && robot.fastColorSensor.getBrightness()<robot.BRIGHTNESS_WHITE_THRESHOLD);
        robot.endSynchronousDriving(this);
        telemetry.addData("turn to beacon 2", "");
        telemetry.update();
        robot.oneWheelTurn(robot.RIGHT_MOTOR, -90, 0.5, this);
        telemetry.addData("follow line", "");
        telemetry.update();
        robot.followLineStraightBlue(0.35, 0.1, this);
        telemetry.addData("get beacon 2", "");
        telemetry.update();
        if(getLeftColor() == robot.BEACON_BLUE){
            robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
        }
        else{
            robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
        }
        robot.driveDistance(0.15, 0.3, this);
        robot.driveDistance(-0.15, 0.4, this);
        telemetry.addData("end", "");
        telemetry.update();

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