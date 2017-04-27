package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;

import java.util.concurrent.locks.Lock;

/**
 * Created by christopher.johnson on 2/19/17.
 */

public class ColorSensorEx extends ModernRoboticsI2cColorSensor {
    public final static I2cAddr DEFAULT_I2C_ADDRESS = I2cAddr.create8bit(0x3C);
    public final static int ADDRESS_COMMAND = 0x03;
    public final static int ADDRESS_COLOR_NUMBER = 0x04;
    public final static int OFFSET_COMMAND = 0x0 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
    public final static int OFFSET_COLOR_NUMBER = 0x01 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
    public final static int OFFSET_RED_READING = 0x02 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
    public final static int OFFSET_GREEN_READING = 0x03 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
    public final static int OFFSET_BLUE_READING = 0x04 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
    public final static int OFFSET_ALPHA_VALUE = 0x05 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
    public final static int BUFFER_LENGTH = 0x06;
    public final static int COMMAND_PASSIVE_LED = 0x01;
    public final static int COMMAND_ACTIVE_LED = 0x00;

    //------------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------------

    private volatile I2cAddr i2cAddr = DEFAULT_I2C_ADDRESS; // this can be changed by the user
    private          byte[]  readBuffer;
    private Lock readLock;
    private          byte[]  writeBuffer;
    private          Lock    writeLock;

    private enum State {READING_ONLY, PERFORMING_WRITE, SWITCHING_TO_READ};
    private State state = State.READING_ONLY;
    private int lastCommand = COMMAND_ACTIVE_LED; // NXT Sensor starts up with the LED on.
    public enum COMMAND {
        ACTIVE_MEASUREMENT,
        PASSIVE_MEASUREMENT,
        OPERATING_FREQUENCY_50HZ,
        OPERATING_FREQUENCY_60HZ,
        BLACK_LEVEL_CALIBRATE,
        WHITE_BALANCE_CALIBRATE
    }

    public ColorSensorEx(I2cController module, int physicalPort) {
        super(module, physicalPort);
    }

    @Override
    protected void controllerNowArmedOrPretending() {
        super.controllerNowArmedOrPretending();
    }

    @Override
    public String toString() {
        return super.toString();
    }

    @Override
    public int red() {
        return super.red();
    }

    @Override
    public int green() {
        return super.green();
    }

    @Override
    public int blue() {
        return super.blue();
    }

    @Override
    public int alpha() {
        return super.alpha();
    }

    @Override
    public int argb() {
        return super.argb();
    }

    @Override
    public synchronized void enableLed(boolean enable) {
        super.enableLed(enable);
    }

    @Override
    public Manufacturer getManufacturer() {
        return super.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return super.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return super.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return super.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        super.close();
    }

    @Override
    public synchronized void portIsReady(int port) {
        super.portIsReady(port);
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        super.setI2cAddress(newAddress);
    }

    @Override
    public I2cAddr getI2cAddress() {
        return super.getI2cAddress();
    }

    public void runCommand(COMMAND cmd) {
        switch (cmd) {
            case ACTIVE_MEASUREMENT:
                enableLed(true);
                break;
            case PASSIVE_MEASUREMENT:
                enableLed(false);
                break;
            case OPERATING_FREQUENCY_50HZ:
                state = State.PERFORMING_WRITE;
                try {
                    writeLock.lock();
                    writeBuffer[OFFSET_COMMAND] = (byte) 0x35;
                } finally {
                    writeLock.unlock();
                }
                break;
            case OPERATING_FREQUENCY_60HZ:
                state = State.PERFORMING_WRITE;
                try {
                    writeLock.lock();
                    writeBuffer[OFFSET_COMMAND] = (byte) 0x36;
                } finally {
                    writeLock.unlock();
                }
                break;
            case BLACK_LEVEL_CALIBRATE:
                state = State.PERFORMING_WRITE;
                try {
                    writeLock.lock();
                    writeBuffer[OFFSET_COMMAND] = (byte) 0x42;
                } finally {
                    writeLock.unlock();
                }
                break;
            case WHITE_BALANCE_CALIBRATE:
                state = State.PERFORMING_WRITE;
                try {
                    writeLock.lock();
                    writeBuffer[OFFSET_COMMAND] = (byte) 0x43;
                } finally {
                    writeLock.unlock();
                }
                break;
        }
    }
}
