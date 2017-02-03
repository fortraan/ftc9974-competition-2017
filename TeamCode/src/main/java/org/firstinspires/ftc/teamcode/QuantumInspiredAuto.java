package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * Created by christopher.johnson on 12/4/16.
 */

@Autonomous(name="Quantum Auto")
public class QuantumInspiredAuto extends LinearOpMode {
    private final double speedOfApproach = 0.3;
    private final double lightThreshold = 0.05;
    private final double turnLeft = -0.4;
    private final double turnRight = 0.4;
    private final long turnTime = 450;

    private final float MARGIN = 2;

    private final float targetAzimuthChange = -60;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive robot = new RobotDrive(RobotDrive.RANGER, hardwareMap);

        /*
        Compass compass = new Compass(hardwareMap.appContext);
        compass.start();
        */

        //LightSensor legoLightSensor = hardwareMap.lightSensor.get("light");
        OpticalDistanceSensor lightSensor = hardwareMap.opticalDistanceSensor.get("line");

        lightSensor.enableLed(true);

        //ColorSensor rightColor = hardwareMap.colorSensor.get("right");
        //ColorSensor leftColor = hardwareMap.colorSensor.get("left");

        DcMotor slide = hardwareMap.dcMotor.get("slide");

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Light (level)", lightSensor.getLightDetected());
            telemetry.addData("Light (raw)", lightSensor.getRawLightDetected());
            telemetry.update();
            idle();
        }
        /*
        float beginningAzimuth = compass.azimuth;
        float targetAzimuth = ((beginningAzimuth + targetAzimuthChange) + 360) % 360;
        float lowerBound = ((targetAzimuth - MARGIN) + 360) % 360;
        float upperBound = ((targetAzimuth + MARGIN) + 360) % 360;
        */

        if (opModeIsActive()) robot.update(speedOfApproach, speedOfApproach);

        while (opModeIsActive() && lightSensor.getLightDetected() < lightThreshold) {
            telemetry.addData("Light (level)", lightSensor.getLightDetected());
            telemetry.addData("Light (raw)", lightSensor.getRawLightDetected());
            telemetry.update();
        }

        robot.update(0, 0);

        sleep(1000);

        /*robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left.setTargetPosition(-54000);
        robot.right.setTargetPosition(54000);*/

        robot.update(-0.3, 0.3);

        /*while ((robot.left.isBusy() || robot.right.isBusy()) && !isStopRequested()) {
            telemetry.addLine("Turning...");
            telemetry.addData("LE", robot.left.getCurrentPosition());
            telemetry.update();
        }*/

        sleep(turnTime);

        robot.update(0, 0);

        /*while (compass.azimuth < lowerBound && compass.azimuth > upperBound) {
            telemetry.addData("Compass azimuth", compass.azimuth);
            telemetry.update();
            idle();
        }*/

        sleep(1000);

        slide.setPower(0.3);
        sleep(1000);
        slide.setPower(-0.3);
        sleep(1000);
        slide.setPower(0);
    }
}
