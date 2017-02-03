package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by christopher.johnson on 11/5/16.
 *
 * Auto that parks on the conrner vortex
 */

@Autonomous(name="Park Auto (RED)", group="2017")
@Disabled
public class ParkAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor left = hardwareMap.colorSensor.get("left");
        ColorSensor right = hardwareMap.colorSensor.get("right");
        RobotDrive rb = new RobotDrive(RobotDrive.RANGER, hardwareMap);

        while (!(left.red() < 200)) {
            rb.update(1, 1);
        }

        while (right.red() < 200) {
            rb.update(0, 1);
        }
    }
}
