package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by christopher.johnson on 11/27/16.
 */

public class LowLevelColorSensor {
    private byte cCache[];
    private I2cDevice cDevice;
    private I2cDeviceSynch cDeviceSynch;

    public LowLevelColorSensor(HardwareMap hardwareMap, @NonNull String nameOfSensor) {
        cDevice = hardwareMap.i2cDevice.get(nameOfSensor);
        cDeviceSynch = new I2cDeviceSynchImpl(cDevice, I2cAddr.create8bit(0x3C), false);
        cDeviceSynch.engage();

        // Turn the LED on
        cDeviceSynch.write8(3, 0);
    }

    public int argb() {
        cCache = cDeviceSynch.read(0x04, 1);
        return cCache[0] & 0xFF;
    }
}
