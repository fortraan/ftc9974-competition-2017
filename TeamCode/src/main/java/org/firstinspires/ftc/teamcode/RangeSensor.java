package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by christopher.johnson on 2/14/17.
 */

@TeleOp(name = "Range")
public class RangeSensor extends OpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void init() {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
    }

    @Override
    public void loop() {
        telemetry.addData("Raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("Raw optical", rangeSensor.rawOptical());
        telemetry.addData("Ultrasonic", rangeSensor.cmUltrasonic());
        telemetry.addData("Optical", rangeSensor.cmOptical());
        telemetry.addData("Overall", rangeSensor.getDistance(DistanceUnit.CM));
    }
}
