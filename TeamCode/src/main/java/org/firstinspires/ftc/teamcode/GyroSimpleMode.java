package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by christopher.johnson on 12/21/16.
 */

@Autonomous(name="Gyro")
public class GyroSimpleMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FTCOrientation orientation = new FTCOrientation(hardwareMap.appContext);
        while (!isStopRequested()) {
            telemetry.addData("Azimuth", (orientation.azimuth == 370) ? "Waiting for reading" : Math.toDegrees(orientation.azimuth));
            telemetry.addData("Pitch", (orientation.pitch == 370) ? "Waiting for reading" : Math.toDegrees(orientation.pitch));
            telemetry.addData("Roll", (orientation.roll == 370) ? "Waiting for reading" : Math.toDegrees(orientation.roll));
            telemetry.update();
        }
    }
}
