package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by christopher.johnson on 12/21/16.
 */

@Autonomous(name="Compass")
public class CompassSimpleMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Compass orientation = new Compass(hardwareMap.appContext);
        orientation.start();
        while (!isStopRequested()) {
            float averageval = 0;
            for (int i = 0; i < 5; i++) {
                averageval += orientation.azimuth;
            }
            float average = (averageval == 0) ? 0 : averageval / 5;
            telemetry.addData("Azimuth", orientation.azimuth);
            telemetry.addData("Average", average);
            telemetry.update();
        }
    }
}
