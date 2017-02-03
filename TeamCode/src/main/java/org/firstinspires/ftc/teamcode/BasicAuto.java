package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.TypeConversion;

/**
 * Created by christopher.johnson on 11/6/16.
 */
@Autonomous(name="Basic Auto",group="2017")
public class BasicAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive rb = new RobotDrive(RobotDrive.RANGER, hardwareMap);
        //ModernRoboticsI2cColorSensor9974ed lineFollower = (ModernRoboticsI2cColorSensor9974ed) hardwareMap.get("line");

        /*
        byte colorCache[];

        I2cDevice colorDevice = hardwareMap.i2cDevice.get("line");
        I2cDeviceSynch colorDeviceSynch = new I2cDeviceSynchImpl(colorDevice, I2cAddr.create8bit(0x3C), false);
        colorDeviceSynch.engage();
        */
        LowLevelColorSensor llcs = new LowLevelColorSensor(hardwareMap, "line");

        TouchSensor stop = hardwareMap.touchSensor.get("stop");

        rb.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lineFollower.enableLed(true);
        //colorDeviceSynch.write8(3, 0);

        waitForStart();

        while (!stop.isPressed() && !super.isStopRequested()) {
            //colorCache = colorDeviceSynch.read(0x04, 1);

            //telemetry.addData("Sensor ARGB", colorCache[0] & 0xFF);
            telemetry.addData("Sensor ARGB", llcs.argb());
            updateTelemetry(telemetry);
            //if ((colorCache[0] & 0xFF) == 16) {
            if (llcs.argb() != 0) {
                rb.update(0.2, 0.6);
            } else {
                rb.update(0.6, 0.2);
            }
        }

        rb.update(0, 0);
    }
}
