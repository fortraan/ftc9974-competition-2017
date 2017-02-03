package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by christopher.johnson on 11/5/16.
 *
 * Beacon auto for blue
 */
@Autonomous(name="Color auto (BLUE)", group="2017")
@SuppressWarnings("unused")
public class ColorAutoBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor distance = (DistanceSensor) hardwareMap.opticalDistanceSensor.get("distance");
        ColorSensor beacon = hardwareMap.colorSensor.get("beacon");
        ColorSensor line = hardwareMap.colorSensor.get("line");
        Servo right = hardwareMap.servo.get("right");
        Servo left = hardwareMap.servo.get("left");
        RobotDrive rb = new RobotDrive(RobotDrive.RANGER, hardwareMap);

        right.setPosition(0);
        left.setPosition(0);

        rb.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rb.update(1, 1);

        while (line.alpha() < 200) {
            sleep(1);
        }
        // Now follow the line, tracking the left side
        while (distance.getDistance(DistanceUnit.INCH) > 2) {
            if (line.alpha() > 200) {
                rb.update(0.9, 1);
            } else {
                rb.update(1, 0.9);
            }
        }

        rb.update(0, 0);

        // This assumes that the sensor is mounted on the right
        if (beacon.blue() > 200) {
            // It's on this side
            telemetry.addLine("Decision: RIGHT");

            right.setPosition(90);
            sleep(2000);
            right.setPosition(0);
            if (beacon.blue() < 200) {
                right.setPosition(90);
                sleep(2000);
                right.setPosition(0);
            }
        } else {
            // It's on the other side
            telemetry.addLine("Decision: LEFT");

            left.setPosition(90);
            sleep(2000);
            left.setPosition(0);
            if (beacon.red() < 200) {
                left.setPosition(90);
                sleep(2000);
                left.setPosition(0);
            }
        }
    }
}
