package org.firstinspires.ftc.teamcode;

import android.widget.Toast;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by christopher.johnson on 12/4/16.
 *
 * Quantum Gyro Auto
 * Designation: Autonomous
 * Purpose: Push both beacons autonomously with the help of multiple sensors
 * @author christopher.johnson AKA Christopher
 * Year/Season: 2017 (Velocity Vortex)
 */

// TODO: 1/16/17 Clean up and optimize code readability

@Autonomous(name="Quantum Gyro Auto")
public class QuantumInspiredGyroAuto extends LinearOpMode {
    // the wheels have a diameter of 4 inches, so a circumference of 4*pi,
    // so at 1440 encoder ticks per revolution of the internal shaft,
    // or 86400 ticks/rev of the output shaft (the drivetrain uses Neverest 60s),
    // that means that (4*pi)/(1440*60) = inches per tick
    private final double inchesPerTick = 0.0001454441;

    private final double speedOfApproach = 0.1;
    private final double lightThreshold = 0.077;
    /*
    private final double turnLeft = -0.4;
    private final double turnRight = 0.4;
    private final long turnTime = 450;

    private final float MARGIN = 2;

    private final float targetAzimuthChange = -60;
    */

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive robot = new RobotDrive(RobotDrive.RANGER, hardwareMap);

        OpticalDistanceSensor lightSensor = hardwareMap.opticalDistanceSensor.get("blue");
        lightSensor.enableLed(true);

        //OpticalDistanceSensor lightSensorLeft = hardwareMap.opticalDistanceSensor.get("lineLeft");
        //lightSensorLeft.enableLed(true);

        DcMotor slide = hardwareMap.dcMotor.get("slide");

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        ModernRoboticsI2cColorSensor beaconColorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("beaconBlue");
        beaconColorSensor.enableLed(true);

        PID gyroPID;

        try {
            gyro.calibrate();
        } catch (UnsupportedOperationException e) {
            Toast.makeText(hardwareMap.appContext, "This gyro doesn't support calibration. :(", Toast.LENGTH_LONG).show();
        }

        while (!(isStarted() || isStopRequested())) {
            if (!gyro.isCalibrating()) {
                telemetry.addData("Raw Z", gyro.rawZ());
                telemetry.addData("Heading", gyro.getHeading());
                telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            }
            telemetry.addData("Light (level)", lightSensor.getLightDetected());
            telemetry.addData("Light (raw)", lightSensor.getRawLightDetected());
            telemetry.update();
            idle();
        }
        beaconColorSensor.enableLed(false);

        if (isStopRequested()) {
            return;
        }

        while(gyro.isCalibrating()) {
            telemetry.addData("Calibrating...", "Calibrating");
            telemetry.update();
            idle();
        }

        // Start moving
        if (opModeIsActive()) robot.update(speedOfApproach, speedOfApproach);

        // Stop when the line is detected
        while (opModeIsActive() && lightSensor.getLightDetected() < lightThreshold) {
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.addData("Light (level)", lightSensor.getLightDetected());
            telemetry.addData("Light (raw)", lightSensor.getRawLightDetected());
            telemetry.update();
        }

        robot.update(0, 0);

        sleep(1000);

        gyro.calibrate();
        // TODO: 1/16/17 Possible PID(s)(?)
        //gyroPID = new PID(-2880, 2880, )
        while (gyro.isCalibrating()) {
            telemetry.addData("Calibrating...", "Calibrating...");
            telemetry.update();
            idle();
        }

        // TODO: 1/16/17 If there will be PIDs, test tunings of Kp=0.35, Ki=0.5, and Kd=0.7
        //gyroPID = new PID(-0.1, 0.1, 60, 0.35, 0.5, 0.7, PID.MODES.MANUAL, gyro.getHeading());

        sleep(1000);

        robot.update(-0.08, 0.08);

        while (opModeIsActive() && gyro.getIntegratedZValue() < 45) {
            // TODO: 1/16/17 Error correction loop?
            /*gyroPID.input = gyro.getHeading();
            if (gyroPID.DoCycle()) {
                robot.update(-gyroPID.output, gyroPID.output);
                telemetry.addData("PID output", gyroPID.output);
            }*/

            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        slide.setPower(0.3);
        sleep(500);
        slide.setPower(0);

        /*
        if (beaconColorSensor.argb() == 3) {
            slide.setPower(0.3);
            sleep(500);
            slide.setPower(0);
            sleep(1000);
            slide.setPower(-0.3);
            sleep(500);
            slide.setPower(0);
        }
        */

        slide.setPower(-0.3);
        sleep(500);
        slide.setPower(0);
    }
}
