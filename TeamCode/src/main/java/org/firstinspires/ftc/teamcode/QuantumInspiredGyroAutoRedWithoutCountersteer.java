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
 * Quantum Gyro Auto without countersteer
 * Designation: Autonomous
 * Purpose: Push both beacons autonomously with the help of multiple sensors
 * @author christopher.johnson AKA Christopher
 * Year/Season: 2017 (Velocity Vortex)
 * Inspired by team 6051 The Quantum Mechanics
 */

// TODO: 1/16/17 Clean up and optimize code readability

@Autonomous(name="Quantum Gyro Auto (Red) (without countersteer)")
public class QuantumInspiredGyroAutoRedWithoutCountersteer extends LinearOpMode {

    /**
     * Gives the number of ticks to go a certain distance as given by this formula:
     * <br/>
     * t = (x * r) / c
     * </br>
     * where
     * t is said number of ticks
     * x is said distance in inches
     * r is the number of pulses/rev. of the motor shaft
     * and c is the circumference of the wheel
     *
     * <p>
     *     The robot uses Neverest 60 motors with 4 inch diameter wheels for a drivetrain. This means that
     *     r = 1680 and c = 4π.
     * </p>
     * @param distance distance to go, in inches
     * @return ticks
     */
    private int ticksForDistance(double distance) {
        return (int) Math.round((distance * 1680) / (4*Math.PI));
    }

    private final double speedOfApproach = 0.25;
    private final double lightThreshold = 0.063;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);

        OpticalDistanceSensor lightSensorRed = hardwareMap.opticalDistanceSensor.get("red");
        lightSensorRed.enableLed(true);
        OpticalDistanceSensor lightSensorBlue = hardwareMap.opticalDistanceSensor.get("blue");
        lightSensorBlue.enableLed(true);

        DcMotor slide = hardwareMap.dcMotor.get("slide");

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        ModernRoboticsI2cColorSensor beaconColorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("beaconRed");
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
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            telemetry.update();
            idle();
        }
        beaconColorSensor.enableLed(false);

        if (isStopRequested()) {
            return;
        }

        while(gyro.isCalibrating() && !isStopRequested()) {
            telemetry.addData("Calibrating...", "Calibrating");
            telemetry.update();
            idle();
        }

        // Start moving
        robot.update(speedOfApproach, speedOfApproach);

        // Stop when the line is detected
        while (!isStopRequested() && lightSensorRed.getLightDetected() < lightThreshold) {
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            telemetry.update();
        }

        robot.update(0, 0);

        if (isStopRequested()) {
            return;
        }

        // TODO: 1/16/17 If there will be PIDs, test tunings of Kp=0.35, Ki=0.5, and Kd=0.7
        //gyroPID = new PID(-0.1, 0.1, 60, 0.35, 0.5, 0.7, PID.MODES.MANUAL, gyro.getHeading());

        sleep(100);

        robot.update(0.16, 0);

        while (!isStopRequested() && gyro.getIntegratedZValue() > -45) {
            // TODO: 1/16/17 Error correction loop?
            /*gyroPID.input = gyro.getHeading();
            if (gyroPID.DoCycle()) {
                robot.update(-gyroPID.output, gyroPID.output);
                telemetry.addData("PID output", gyroPID.output);
            }*/

            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMotorTargets(robot.left.getCurrentPosition()-ticksForDistance(11.5), robot.right.getCurrentPosition()-ticksForDistance(11.5));

        robot.update(0.5, 0.5);

        while (!isStopRequested() && robot.areMotorsBusy(true)) {
            telemetry.addData("Integrated heading", gyro.getIntegratedZValue());
            telemetry.addData("Left encoder", robot.left.getCurrentPosition());
            telemetry.addData("Right encoder", robot.right.getCurrentPosition());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);


        slide.setPower(0.3);
        sleep(500);
        slide.setPower(0);

        if (beaconColorSensor.argb() == 10) {
            slide.setPower(0.3);
            sleep(500);
            slide.setPower(0);
            sleep(1000);
            slide.setPower(-0.3);
            sleep(1000);
            slide.setPower(0);
        } else {
            slide.setPower(-0.3);
            sleep(500);
            slide.setPower(0);
            robot.setMotorTargets(robot.left.getCurrentPosition() - ticksForDistance(6), robot.right.getCurrentPosition() - ticksForDistance(6));
            robot.update(0.5, 0.5);
            while (!isStopRequested() && robot.areMotorsBusy(true)) {
                telemetry.addData("Integrated heading", gyro.getIntegratedZValue());
                telemetry.addData("Left encoder", robot.left.getCurrentPosition());
                telemetry.addData("Right encoder", robot.right.getCurrentPosition());
                telemetry.update();
                idle();
            }
            robot.update(0, 0);
            slide.setPower(0.3);
            sleep(1000);
            slide.setPower(0);
            sleep(1000);
            slide.setPower(-0.3);
            sleep(1000);
            slide.setPower(0);
        }
    }
}
