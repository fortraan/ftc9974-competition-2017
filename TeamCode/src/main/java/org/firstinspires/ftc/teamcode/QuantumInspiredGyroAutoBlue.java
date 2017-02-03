package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by christopher.johnson on 12/4/16.
 *
 * Quantum Gyro Auto
 * Designation: Autonomous
 * Purpose: Push both beacons autonomously with the help of multiple sensors
 * @author christopher.johnson AKA Christopher
 * Year/Season: 2017 (Velocity Vortex)
 * Inspired by team 6051 The Quantum Mechanics
 */

// TODO: 1/16/17 Clean up and optimize code readability

@Autonomous(name="Quantum Gyro Auto (Blue)")
public class QuantumInspiredGyroAutoBlue extends LinearOpMode {

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

    private final double speedOfApproach = 0.6;
    private final double lightThreshold = 0.067;
    final int ticksForSlide = 5340;
    final int redThreshold = 16;
    final int blueThreshold = 32;

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

        ModernRoboticsI2cColorSensor beaconColorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("beaconBlue");
        //beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x22));
        beaconColorSensor.enableLed(true);

        //PID gyroPID;

        TouchSensor stop = hardwareMap.touchSensor.get("stop");

        gyro.calibrate();

        float hsvVals[] = {0, 0, 0};

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
            Color.RGBToHSV(beaconColorSensor.red() * 8, beaconColorSensor.green() * 8, beaconColorSensor.blue() * 8, hsvVals);
            telemetry.addData("H", hsvVals[0]);
            telemetry.addData("S", hsvVals[1]);
            telemetry.addData("V", hsvVals[2]);
            telemetry.addData("R", beaconColorSensor.red() * 8);
            telemetry.addData("G", beaconColorSensor.green() * 8);
            telemetry.addData("B", beaconColorSensor.blue() * 8);
            telemetry.addData("Color", beaconColorSensor.argb());
            //telemetry.addData("Color", Color.HSVToColor(hsvVals));
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

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMotorTargets(-ticksForDistance(2.5), -ticksForDistance(2.5));

        robot.update(0.5, 0.5);

        while (robot.areMotorsBusy(true) && !isStopRequested()) {
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.update(0.16, -0.16);

        while (!isStopRequested() && /*!(gyro.getIntegratedZValue() > 44 && */gyro.getIntegratedZValue() > -43)/*)*/ {
            // TODO: 1/16/17 Error correction loop?
            /*gyroPID.input = gyro.getHeading();
            if (gyroPID.DoCycle()) {
                robot.update(-gyroPID.output, gyroPID.output);
                telemetry.addData("PID output", gyroPID.output);
            }*/

            /*if (gyro.getIntegratedZValue() < 44) {
                robot.update(-0.16, 0.16);
            } else {
                robot.update(0.16, -0.6);
            }*/

            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        robot.update(speedOfApproach, speedOfApproach);

        if (!stop.isPressed()) slide.setPower(-0.2);

        // Stop when the line is detected
        boolean tempReset = false;
        while (!isStopRequested() && lightSensorRed.getLightDetected() < lightThreshold) {
            if (stop.isPressed() && !tempReset) {
                slide.setPower(0);

                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slide.setTargetPosition((int) (ticksForSlide * 0.5));
                slide.setPower(0.3);
                tempReset = true;
            }
            if (tempReset && !slide.isBusy()) {
                // Slide has reset
                slide.setPower(0);
            }
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        while (slide.isBusy()) {
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        if (isStopRequested()) {
            return;
        }

        // TODO: 1/16/17 If there will be PIDs, test tunings of Kp=0.35, Ki=0.5, and Kd=0.7
        //gyroPID = new PID(-0.1, 0.1, 60, 0.35, 0.5, 0.7, PID.MODES.MANUAL, gyro.getHeading());

        sleep(50);

        robot.update(-0.16, 0.16);

        while (!isStopRequested() && gyro.getIntegratedZValue() < 0) {
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
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
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
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        //sleep(1000);

        /*slide.setPower(-0.3);
        sleep(500);
        slide.setPower (0.3);
        sleep(500);
        slide.setPower(0);

        if (beaconColorSensor.argb() == 10) {
            slide.setPower(-0.3);
            sleep(500);
            slide.setPower(0);
            sleep(1000);
            slide.setPower(0.3);
            sleep(1000);
            slide.setPower(0);
        } else {
            slide.setPower(0.3);
            sleep(500);
            slide.setPower(0);
            robot.setMotorTargets(robot.left.getCurrentPosition() - ticksForDistance(6), robot.right.getCurrentPosition() - ticksForDistance(6));
            robot.update(0.5, 0.5);
            while (!isStopRequested() && robot.areMotorsBusy(true)) {
                telemetry.addData("Integrated heading", gyro.getIntegratedZValue());
                telemetry.addData("Left encoder", robot.left.getCurrentPosition());
                telemetry.addData("Right encoder", robot.right.getCurrentPosition());
                telemetry.addData("Color sensor", beaconColorSensor.argb());
                telemetry.update();
                idle();
            }
            robot.update(0, 0);
            slide.setPower(-0.3);
            sleep(1000);
            slide.setPower(0);
            sleep(1000);
            slide.setPower(0.3);
            sleep(1000);
            slide.setPower(0);
        }*/

        slide.setTargetPosition((int) (ticksForSlide * 0.85));

        slide.setPower(1);

        while (!isStopRequested() && slide.isBusy()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.addData("R, G, B", "%d, %d, %d",
                    beaconColorSensor.red()*8,
                    beaconColorSensor.green()*8,
                    beaconColorSensor.blue()*8
            );
            telemetry.update();
            idle();
        }

        slide.setPower(0);

        //sleep(1000);

        if (beaconColorSensor.red()*8 < redThreshold && beaconColorSensor.blue()*8 > blueThreshold) {
            slide.setTargetPosition(ticksForSlide);
            slide.setPower(0.5);
            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red()*8,
                        beaconColorSensor.green()*8,
                        beaconColorSensor.blue()*8
                );
                telemetry.update();
                idle();
            }
            slide.setPower(0);
            sleep(3000);
            slide.setTargetPosition((int) (ticksForSlide * 0.5));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red()*8,
                        beaconColorSensor.green()*8,
                        beaconColorSensor.blue()*8
                );
                telemetry.update();
                idle();
            }

            slide.setPower(0);
        } else {
            slide.setTargetPosition((int) (ticksForSlide * 0.5));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red() * 8,
                        beaconColorSensor.green() * 8,
                        beaconColorSensor.blue() * 8
                );
                telemetry.update();
                idle();
            }

            slide.setPower(0);

            robot.setMotorTargets(robot.left.getCurrentPosition() - ticksForDistance(6), robot.right.getCurrentPosition() - ticksForDistance(6));

            robot.update(0.2, 0.2);

            while (!isStopRequested() && robot.areMotorsBusy(true)) {
                telemetry.addData("Integrated heading", gyro.getIntegratedZValue());
                telemetry.addData("Left encoder", robot.left.getCurrentPosition());
                telemetry.addData("Right encoder", robot.right.getCurrentPosition());
                //telemetry.addData("Color sensor", beaconColorSensor.argb());
                telemetry.update();
                idle();
            }

            robot.update(0, 0);

            sleep(500);

            slide.setTargetPosition(ticksForSlide);
            slide.setPower(0.5);
            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red() * 8,
                        beaconColorSensor.green() * 8,
                        beaconColorSensor.blue() * 8
                );
                telemetry.update();
                idle();
            }
            slide.setPower(0);
            sleep(3000);
            slide.setTargetPosition((int) (ticksForSlide * 0.5));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red() * 8,
                        beaconColorSensor.green() * 8,
                        beaconColorSensor.blue() * 8
                );
                telemetry.update();
                idle();
            }

            slide.setPower(0);
        }

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.update(speedOfApproach, speedOfApproach);

        while (!isStopRequested() && lightSensorRed.getLightDetected() < lightThreshold) {
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMotorTargets(robot.left.getCurrentPosition()-ticksForDistance(9.5), robot.right.getCurrentPosition()-ticksForDistance(9.5));

        robot.update(0.5, 0.5);

        while (!isStopRequested() && robot.areMotorsBusy(true)) {
            telemetry.addData("Integrated heading", gyro.getIntegratedZValue());
            telemetry.addData("Left encoder", robot.left.getCurrentPosition());
            telemetry.addData("Right encoder", robot.right.getCurrentPosition());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        slide.setTargetPosition((int) (ticksForSlide * 0.85));

        slide.setPower(1);

        while (!isStopRequested() && slide.isBusy()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.addData("R, G, B", "%d, %d, %d",
                    beaconColorSensor.red()*8,
                    beaconColorSensor.green()*8,
                    beaconColorSensor.blue()*8
            );
            telemetry.update();
            idle();
        }

        slide.setPower(0);

        //sleep(1000);

        if (beaconColorSensor.red()*8 < redThreshold && beaconColorSensor.blue()*8 > blueThreshold) {
            slide.setTargetPosition(ticksForSlide);
            slide.setPower(0.5);
            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red()*8,
                        beaconColorSensor.green()*8,
                        beaconColorSensor.blue()*8
                );
                telemetry.update();
                idle();
            }
            slide.setPower(0);
            sleep(3000);
            slide.setTargetPosition((int) (ticksForSlide * 0.5));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red()*8,
                        beaconColorSensor.green()*8,
                        beaconColorSensor.blue()*8
                );
                telemetry.update();
                idle();
            }

            slide.setPower(0);
        } else {
            slide.setTargetPosition((int) (ticksForSlide * 0.5));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red() * 8,
                        beaconColorSensor.green() * 8,
                        beaconColorSensor.blue() * 8
                );
                telemetry.update();
                idle();
            }

            slide.setPower(0);

            robot.setMotorTargets(robot.left.getCurrentPosition() - ticksForDistance(6), robot.right.getCurrentPosition() - ticksForDistance(6));

            robot.update(0.2, 0.2);

            while (!isStopRequested() && robot.areMotorsBusy(true)) {
                telemetry.addData("Integrated heading", gyro.getIntegratedZValue());
                telemetry.addData("Left encoder", robot.left.getCurrentPosition());
                telemetry.addData("Right encoder", robot.right.getCurrentPosition());
                //telemetry.addData("Color sensor", beaconColorSensor.argb());
                telemetry.update();
                idle();
            }

            robot.update(0, 0);

            sleep(500);

            slide.setTargetPosition(ticksForSlide);
            slide.setPower(0.5);
            while (!isStopRequested() && slide.isBusy() && !stop.isPressed()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red() * 8,
                        beaconColorSensor.green() * 8,
                        beaconColorSensor.blue() * 8
                );
                telemetry.update();
                idle();
            }
            slide.setPower(0);
            sleep(3000);
            slide.setTargetPosition((int) (ticksForSlide * 0.5));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy()) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red() * 8,
                        beaconColorSensor.green() * 8,
                        beaconColorSensor.blue() * 8
                );
                telemetry.update();
                idle();
            }

            slide.setPower(0);
        }
    }
}
