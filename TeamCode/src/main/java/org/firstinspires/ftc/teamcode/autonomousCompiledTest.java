package org.firstinspires.ftc.teamcode;



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
public class autonomousCompiledTest extends LinearVisionOpMode {
    private double redTolerance = 0;
    private double blueTolerance = 0;

    HardwareMapLucyV4 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //start everything with the boot up
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero(this);
        //end every boot by resetting the motors
        beginDetection(robot.RED_THREASHOLD, robot.BLUE_THREASHOLD,robot.CAMERA_FRAME_HEIGHT, robot.CAMERA_FRAME_WIDTH);
        waitForStart();
        robot.driveDistance(3, 1);
        robot.turnToDegree(45);
        robot.goForwardUntilWhite(robot.USE_BRIGHTNESS);
        double dist = robot.distSensor.getLightDetected();
        while(dist>.1 && opModeIsActive()){
            robot.driveDistance(.05,.1);
            dist = robot.distSensor.getLightDetected();
        }
        robot.brakeTemporarily();
        robot.deployBeaconPressers();
        //telemetry.addData("Color Left:",getRightColor());
        if(getRightColor()==robot.BEACON_RED){
            robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
            telemetry.addData("Red On:","Right");
            //robot.driveDistance(.04, 1);
        }
        if(getLeftColor()==robot.BEACON_RED){
            robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
            telemetry.addData("Red On:","Left");
            //robot.driveDistance(.04, 1);
        }
        robot.driveDistance(.02, 1);
        robot.driveDistance(5, -1);
        robot.turnToDegree(135);
        robot.driveDistance(5, 1);
        //robot.flyWheel1.setPower(robot.FLY_WHEEL_POWER);
        //robot.flyWheel2.setPower(robot.FLY_WHEEL_POWER);
        //robot.indexer.setPosition(robot.INDEXER_FIRE_POSITION);
        //robot.indexer.setPosition(robot.INDEXER_LOAD_POSITION);
        telemetry.update();
    }


    public void beginDetection(double redTolerance, double blueTolerance, int height, int width){
        this.redTolerance = redTolerance;
        this.blueTolerance = blueTolerance;
        telemetry.addData("Vars", "Set");
        telemetry.update();
        try {
            waitForVisionStart();
        }
        catch (Exception e){
            telemetry.addData("Exception: ", e.getMessage());
        }

        super.init();
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
        beacon.setColorToleranceRed(redTolerance);
        beacon.setColorToleranceBlue(blueTolerance);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
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
