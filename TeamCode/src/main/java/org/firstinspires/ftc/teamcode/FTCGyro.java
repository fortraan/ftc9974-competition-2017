package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Created by christopher.johnson on 12/5/16.
 */

public class FTCGyro implements SensorEventListener {
    private SensorManager manager;
    private Sensor gyro;

    public volatile float deltaRad[] = new float[3];
    public volatile float deltaDeg[] = new float[3];

    public FTCGyro(Context context) {
        manager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        gyro = manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        manager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        synchronized (this) {
            if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                deltaRad = event.values;
                deltaDeg[0] = (float) Math.toDegrees((double) deltaRad[0]);
                deltaDeg[1] = (float) Math.toDegrees((double) deltaRad[1]);
                deltaDeg[2] = (float) Math.toDegrees((double) deltaRad[2]);
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
