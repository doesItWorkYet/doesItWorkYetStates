package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Created by root on 12/19/16.
 */
public class Orientation implements SensorEventListener {
    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];
    SensorManager sensorService;
    Sensor mag;
    Sensor accel;

    Orientation(SensorManager manager){
        this.sensorService = manager;
        this.accel = sensorService.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        this.mag = sensorService.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        sensorService.registerListener(this, accel,SensorManager.SENSOR_DELAY_NORMAL);
        sensorService.registerListener(this, mag,SensorManager.SENSOR_DELAY_NORMAL);
    }

    public double[] getOrientation(){
        //returns in an array [z,x,y]
        // Rotation matrix based on current readings from accelerometer and magnetometer.
        // Rotation matrix based on current readings from accelerometer and magnetometer.
        final float[] rotationMatrix = new float[9];
        float I[] = new float[9];
        sensorService.getRotationMatrix(rotationMatrix, I, accelerometerReading, magnetometerReading);
        // Express the updated rotation matrix as three Orientation angles.
        final float[] orientationAngles = new float[3];
        sensorService.getOrientation(rotationMatrix, orientationAngles);
        final double[] myOrientaion = {(double) (Math.toDegrees(orientationAngles[0])+360)%360,(Math.toDegrees(orientationAngles[1])+360)%360,(Math.toDegrees(orientationAngles[2])+360)%360};
        return myOrientaion;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
            //Log.d("Accel", "1");
            System.arraycopy(event.values, 0, accelerometerReading,0, accelerometerReading.length);
        }
        if(event.sensor.getType()==Sensor.TYPE_MAGNETIC_FIELD){
            System.arraycopy(event.values, 0, magnetometerReading, 0, magnetometerReading.length);
            //Log.d("MAG", "1");
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }


}

