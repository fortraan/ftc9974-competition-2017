package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Created by christopher.johnson on 12/18/16.
 */

public class FTCOrientation implements SensorEventListener {
    float mGravity[];
    float mGeomagnetic[];
    volatile float azimuth = 370;
    volatile float pitch = 370;
    volatile float roll = 370;

    public FTCOrientation(Context context) {
        SensorManager manager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        Sensor accelerometer = manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor magneticField = manager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        manager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, magneticField, SensorManager.SENSOR_DELAY_FASTEST);
    }

    private float R[] = new float[9];
    private float I[] = new float[9];
    private float orientation[] = new float[3];
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
            mGravity = event.values;
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
            mGeomagnetic = event.values;
        if (mGravity != null && mGeomagnetic != null) {
            this.R = new float[9];
            this.I = new float[9];
            boolean success = SensorManager.getRotationMatrix(this.R, this.I, mGravity, mGeomagnetic);
            if (success) {
                this.orientation = new float[3];
                SensorManager.getOrientation(this.R, this.orientation);
                azimuth = this.orientation[0]; // orientation contains: azimuth, pitch and roll
                pitch = this.orientation[1];
                roll = this.orientation[2];
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
