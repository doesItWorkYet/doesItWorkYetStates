package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Point;
import org.opencv.core.Size;

@Autonomous(name="Red team get beacon one", group="Testing")  // @Autonomous(...) is the other common choice

public class goToFirstBeaconRed extends LinearVisionOpMode {
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

        this.setCamera(Cameras.SECONDARY);
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
        rotation.setIsUsingSecondaryCamera(true);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        waitForStart();
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + 5000 && opModeIsActive()){
            waitOneFullHardwareCycle();
        }
        robot.calibrateGyro(this);
        robot.driveDistanceFollowingHeading(0, 0.2, 0.15, 1, this);
        robot.brakeTemporarily(this);
        telemetry.addData("Turn: ", "Starting");
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.RIGHT_MOTOR, -48, 0.5, 0.05, robot.TURNING_P, 2, this);
        telemetry.addData("Turn: ", "Done");
        telemetry.update();
        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitOneFullHardwareCycle();
        robot.driveDistanceFollowingHeading(-48, 0.8, 0.75, 3, this);
        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitOneFullHardwareCycle();
        robot.followingHeadingToWhiteLine(-48, 0.2, 0.15, this);
        robot.brakeTemporarily(this);
        robot.driveDistanceFollowingHeading(-88, .3,.25,.5,this);
        robot.brakeTemporarily(this);
        telemetry.addData("heading," ,robot.gyro.getIntegratedZValue());
        telemetry.update();
        robot.turnToHeadingProportionalControl(robot.LEFT_MOTOR, -88, .5, .05, robot.TURNING_P, 1, this);
        robot.brakeTemporarily(this);


        telemetry.addData("heading," ,robot.gyro.getIntegratedZValue());
        telemetry.update();


        robot.beginSynchronousDriving(1,.5);
        while(opModeIsActive() && robot.distSensor.getLightDetected() < robot.OPTICAL_SENSOR_THRESHOLD){
            waitOneFullHardwareCycle();
        }
        robot.endSynchronousDriving(this);
        robot.brakeTemporarily(this);

        //deploying code


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
        robot.setDriveMotorPower(.3);
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() < start + 600){
            waitOneFullHardwareCycle();
        }
        robot.stop();
        robot.setDriveMotorPower(-.3);
        start = System.currentTimeMillis();
        while(System.currentTimeMillis() < start + 300){
            waitOneFullHardwareCycle();
        }
        robot.stop();
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