package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

@Autonomous(name="Beacon Pressers Get Red", group="Testing")  // @Autonomous(...) is the other common choice
@Disabled
public class getBeaconTestUpdated2017 extends LinearVisionOpMode {
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

        this.setCamera(Cameras.PRIMARY);
        telemetry.addData("Cameras", "Set");
        telemetry.update();
        this.setFrameSize(new Size(2000, 2000));
        telemetry.addData("Frame", "set");
        telemetry.update();
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        waitForStart();
        robot.deployBeaconPressers();
        Thread.sleep(100);
        waitOneFullHardwareCycle();
        for(int i = 0; i < 3; i ++){
            int colorLeft = getLeftColor();
        }
        int leftColor = getLeftColor();
        //THIS IS REVERSED
        if(leftColor == robot.BEACON_RED){
            telemetry.addData("Left", " red");
            telemetry.update();
            robot.beaconPresserRight.setPosition(robot.BEACON_PRESSER_RIGHT_STORE_POSITION/180.0);
        }
        else{
            telemetry.addData("right", " red");
            telemetry.update();
            robot.beaconPresserLeft.setPosition(robot.BEACON_PRESSER_LEFT_STORE_POSITION/180.0);
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