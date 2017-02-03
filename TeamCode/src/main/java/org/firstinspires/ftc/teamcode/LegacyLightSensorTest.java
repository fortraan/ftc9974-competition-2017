package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by christopher.johnson on 12/4/16.
 */

@Autonomous(name="Legacy")
public class LegacyLightSensorTest extends LinearOpMode {
    private final double speedOfApproach = 0.3;
    private final double lightThreshold = 0.081;
    private final double turnBasePower = 0.2;
    private final double turnRatio = 3;
    private final long turnTime = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive robot = new RobotDrive(RobotDrive.RANGER, hardwareMap);
        //LightSensor legoLightSensor = hardwareMap.lightSensor.get("light");
        OpticalDistanceSensor legoLightSensor = hardwareMap.opticalDistanceSensor.get("light");

        legoLightSensor.enableLed(true);

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Light (level)", legoLightSensor.getLightDetected());
            telemetry.addData("Light (raw)", legoLightSensor.getRawLightDetected());
            telemetry.update();
            idle();
        }

        robot.update(speedOfApproach, speedOfApproach);

        while (opModeIsActive() && legoLightSensor.getLightDetected() < lightThreshold) {
            telemetry.addData("Light (level)", legoLightSensor.getLightDetected());
            telemetry.addData("Light (raw)", legoLightSensor.getRawLightDetected());
            telemetry.update();
        }

        robot.update(0, 0);

        sleep(1000);

        robot.update(turnBasePower, turnBasePower*turnRatio);

        sleep(turnTime);

        robot.update(0, 0);
    }
}
