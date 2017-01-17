package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

/**
 * Created by root on 12/23/16.
 */
@Autonomous(name="Beacon Color Recognition Arms", group="Testing")  // @Autonomous(...) is the other common choice
//@Disabled


public class beaconColorRecognitionArms extends LinearVisionOpMode {
    private double redTolerance = 0;
    private double blueTolerance = 0;

    HardwareMapLucyV4 robot;



    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero();
        telemetry.addData("Vars", "Set");
        telemetry.update();
        try {
            waitForVisionStart();
        }
        catch (Exception e){
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
        this.setFrameSize(new Size(height, width));
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
        //beginDetection(robot.RED_THREASHOLD, robot.BLUE_THREASHOLD,robot.CAMERA_FRAME_HEIGHT, robot.CAMERA_FRAME_WIDTH);
        waitForStart();
        robot.deployBeaconPressers();
        getLeftColor();
        robot.delay(1000);
        //while(opModeIsActive()){
        while(robot.distSensor.getLightDetected()<.1 && opModeIsActive()){
            robot.driveDistance(.05, .5);
        }
        robot.brakeTemporarily();
           if(getLeftColor() == robot.BEACON_RED) {
               robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
               telemetry.addData("Red on:", "Left");
               telemetry.update();
           }
            if(getRightColor() == robot.BEACON_RED) {
                robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
                telemetry.addData("Red on:", "Right");
                telemetry.update();
            }
        //}

        //telemetry.update();
    }


    public void beginDetection(double redTolerance, double blueTolerance, int height, int width){
        this.redTolerance = redTolerance;
        this.blueTolerance = blueTolerance;

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
