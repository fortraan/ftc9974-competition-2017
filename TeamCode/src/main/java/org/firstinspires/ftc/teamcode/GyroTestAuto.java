package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by christopher.johnson on 12/18/16.
 */

@Autonomous(name="Orientation test", group="2017")
public class GyroTestAuto extends LinearOpMode {
    private final double speedOfApproach = 0.3;
    private final double lightThreshold = 0.065;
    private final double turn = -0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        float average = 0;
        RobotDrive robot = new RobotDrive(RobotDrive.RANGER, hardwareMap);
        FTCOrientation orientation = new FTCOrientation(hardwareMap.appContext);
        OpticalDistanceSensor lightSensor = hardwareMap.opticalDistanceSensor.get("line");

        lightSensor.enableLed(true);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Azimuth", (orientation.azimuth == 370) ? "Waiting for reading" : Math.toDegrees(orientation.azimuth) + 180);
            telemetry.addData("Pitch", (orientation.pitch == 370) ? "Waiting for reading" : Math.toDegrees(orientation.pitch) + 180);
            telemetry.addData("Roll", (orientation.roll == 370) ? "Waiting for reading" : Math.toDegrees(orientation.roll) + 180);
            float averageval = 0;
            for (int i = 0; i < 5; i++) {
                averageval += (float) (Math.toDegrees(orientation.pitch));
            }
            average = (averageval == 0) ? 0 : averageval / 5;
            telemetry.addData("Average", average);
            telemetry.update();
        }

        float startpitch = (float) (Math.toDegrees(orientation.pitch));
        robot.update(0.3, 0.3);

        while (opModeIsActive() && lightSensor.getLightDetected() < lightThreshold) {
            telemetry.addData("Light (level)", lightSensor.getLightDetected());
            telemetry.addData("Light (raw)", lightSensor.getRawLightDetected());
            telemetry.update();/*
            float averageval = 0;
            for (int i = 0; i < 5; i++) {
                averageval += (float) (Math.toDegrees(orientation.azimuth));
            }
            float average = (averageval == 0) ? 0 : averageval / 5;
            if ((average + 5) >= startAzimuth /* too far right ) {
                robot.update(speedOfApproach - 0.1, speedOfApproach);
            } else if ((average - 5) >= startAzimuth /* too far left ) {
                robot.update(speedOfApproach, speedOfApproach - 0.1);
            }*/
            robot.update(speedOfApproach, speedOfApproach);
        }

        robot.update(0, 0);

        sleep(1000);

        // the pitch increases going clockwise
        robot.update(-0.25, 0.25);
        /*
        do {
            float averageval = 0;
            for (int i = 0; i < 5; i++) {
                averageval += (float) (Math.toDegrees(orientation.azimuth));
            }
            average = (averageval == 0) ? 0 : averageval / 5;
            telemetry.addData("Average", average);
        } while (opModeIsActive() && (average >= (startAzimuth + turn)));*/
        while (opModeIsActive() && Math.toDegrees(orientation.pitch) > (startpitch + turn)) {
            telemetry.addData("Pitch", Math.toDegrees(orientation.pitch));
            telemetry.update();
            idle();
        }
        robot.update(0, 0);
    }
}
