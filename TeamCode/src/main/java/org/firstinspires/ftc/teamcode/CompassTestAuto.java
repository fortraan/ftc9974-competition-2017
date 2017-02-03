package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by christopher.johnson on 12/18/16.
 */

@Autonomous(name="Compass test", group="2017")
public class CompassTestAuto extends LinearOpMode {
    private final double speedOfApproach = 0.3;
    private final double lightThreshold = 0.065;
    private final double turn = -60;
    @Override
    public void runOpMode() throws InterruptedException {
        float average = 0;
        RobotDrive robot = new RobotDrive(RobotDrive.RANGER, hardwareMap);
        Compass orientation = new Compass(hardwareMap.appContext);
        OpticalDistanceSensor lightSensor = hardwareMap.opticalDistanceSensor.get("line");

        lightSensor.enableLed(true);

        orientation.start();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Azimuth", (orientation.azimuth == 370) ? "Waiting for reading" : orientation.azimuth);
            float averageval = 0;
            for (int i = 0; i < 5; i++) {
                averageval += (float) (orientation.azimuth);
            }
            average = (averageval == 0) ? 0 : averageval / 5;
            telemetry.addData("Average", average);
            telemetry.update();
        }

        float startazimuth = (float) (orientation.azimuth);
        robot.update(0.3, 0.3);

        while (opModeIsActive() && lightSensor.getLightDetected() < lightThreshold) {
            telemetry.addData("Light (level)", lightSensor.getLightDetected());
            telemetry.addData("Light (raw)", lightSensor.getRawLightDetected());
            telemetry.update();/*
            float averageval = 0;
            for (int i = 0; i < 5; i++) {
                averageval += (float) (orientation.azimuth);
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

        robot.update(-0.15, 0.15);
        /*
        do {
            float averageval = 0;
            for (int i = 0; i < 5; i++) {
                averageval += (float) (orientation.azimuth);
            }
            average = (averageval == 0) ? 0 : averageval / 5;
            telemetry.addData("Average", average);
        } while (opModeIsActive() && (average >= (startAzimuth + turn)));*/
        while (opModeIsActive() && orientation.azimuth > ((startazimuth + turn) + 360) % 360) {
            telemetry.addData("azimuth", orientation.azimuth);
            telemetry.update();
            idle();
        }
        robot.update(0, 0);
    }
}
