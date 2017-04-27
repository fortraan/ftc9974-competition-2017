package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by christopher.johnson on 1/16/17.
 * <p>
 * A template for Autos
 * <p>
 * Template instantiated by christopher.johnson
 */

@Autonomous(name = "Beacon (Red)", group = "2017")
@SuppressWarnings("FieldCanBeLocal")
public class BeaconAuto2Red extends LinearOpMode {

    /**
     * Gives the number of ticks to go a certain distance as given by this formula:
     * <br/>
     * {@code t = (x * r) / c }
     * </br>
     * where
     * t is said number of ticks
     * x is said distance in inches
     * r is the number of pulses/rev. of the motor shaft
     * and c is the circumference of the wheel
     *
     * <p>
     *     The robot uses Neverest 40 motors with 4 inch diameter wheels for a drivetrain. This means that
     *     r = 1120 and c = 4π.
     * </p>
     * @param distance distance to go, in inches
     * @return ticks
     */
    @SuppressWarnings("WeakerAccess")
    protected int ticksForDistance(double distance) {
        return (int) Math.round((distance * 1120) / (4*Math.PI));
    }

    private final double speedOfApproach = 0.8;
    private final int ticksForSlide = 5340;
    private final int redThreshold = 32;
    private final int blueThreshold = 16;
    // --Commented out by Inspection (2/19/17, 11:52):final double Kp = 0.02;

    private final double turnPower = -0.25;
    private final int turnTime = 1500;
    private final double firstTurn = 0.4;
    private final double firstBeaconPushPosition = 0.5;
    private final double firstBeaconButtonDriveDistance = 8.25;
    private final double backDriveTurnDistance = 20;
    private final double secondTurn = -0.38;
    private final double secondBeaconCheckPosition = 0.75;
    private final double secondBeaconButtonDriveDistance = 7.5;
    private final double secondInitialDrive = 1.7;

    @Override
    public void runOpMode() throws InterruptedException {
        // Object oriented robot programming
        RobotDrive robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);

        OpticalDistanceSensor lightSensorRed = hardwareMap.opticalDistanceSensor.get("red");
        lightSensorRed.enableLed(true);
        OpticalDistanceSensor lightSensorBlue = hardwareMap.opticalDistanceSensor.get("blue");
        lightSensorBlue.enableLed(true);

        DcMotor slide = hardwareMap.dcMotor.get("slide");

        ModernRoboticsI2cColorSensor beaconColorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("beaconRed");
        //beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x22));
        beaconColorSensor.enableLed(false);

        //PID gyroPID;

        TouchSensor stop = hardwareMap.touchSensor.get("stop");
        TouchSensor fl = hardwareMap.touchSensor.get("fl");

        ModernRoboticsI2cRangeSensor opticalStop = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        float hsvVals[] = {0, 0, 0};

        ElapsedTime timeout = new ElapsedTime();

        while (!(isStarted() || isStopRequested())) {
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
            telemetry.addData("D0", stop.isPressed());
            telemetry.addData("D1", fl.isPressed());
            telemetry.addData("D1 (Raw)", fl.getValue());
            //telemetry.addData("Ultrasonic", turnStop.getUltrasonicLevel());
            //telemetry.addData("ODS", fr.getLightDetected());
            //telemetry.addData("Color", Color.HSVToColor(hsvVals));
            telemetry.update();
            idle();
        }
        beaconColorSensor.enableLed(false);
        // Find threshold that accounts for ambient light
        double lightThreshold = lightSensorRed.getLightDetected() + 0.01;

        if (isStopRequested()) {
            return;
        }

        // Init encoders
        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start driving forward
        robot.update(speedOfApproach, speedOfApproach);

        // Start centering the slide
        if (!stop.isPressed()) slide.setPower(-0.2);

        // Stop when the line is detected
        boolean tempReset = false;
        while (!isStopRequested() && !fl.isPressed()) {
            telemetry.addData("encoder", robot.left.getCurrentPosition());
            if (stop.isPressed() && !tempReset) {
                // Start moving the slide to home
                slide.setPower(0);

                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slide.setTargetPosition((int) (ticksForSlide * 0.75));
                slide.setPower(0.3);
                tempReset = true;
            }
            if (tempReset && !slide.isBusy()) {
                // Slide has reset
                slide.setPower(0);
            }

            // Slow down after moving 60 inches
            if (robot.left.getCurrentPosition() < -ticksForDistance(60) || robot.right.getCurrentPosition() < -ticksForDistance(60)) {
                robot.update(0.4, 0.4);
            }
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            telemetry.addData("Threshold", lightThreshold);
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

//        robot.update(0, 0);

        // Start turning into the wall
        robot.update(0, turnPower);

        sleep(turnTime);

//        while (!isStopRequested() && turnStop.getUltrasonicLevel() > 5) {
//            idle();
//        }

        // Drive backwards, turning slightly into the wall
        robot.update(firstTurn, 0.4);

        // Keep driving until a line is detected
        while (!isStopRequested() && (lightSensorRed.getLightDetected() + lightSensorBlue.getLightDetected()) / 2 < lightThreshold) {
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            telemetry.addData("Threshold", lightThreshold);
            telemetry.addData("Encoder", slide.getCurrentPosition());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        // Stop
        robot.update(0, 0);

        sleep(500);

        // Turn on encoders
        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Drive forward to the first button
        robot.setMotorTargets(robot.left.getCurrentPosition()-ticksForDistance(12), robot.right.getCurrentPosition()-ticksForDistance(12));

        robot.update(0.2, 0.2);

        while (!isStopRequested() && robot.areMotorsBusy(true)) {
            telemetry.addData("Left encoder", robot.left.getCurrentPosition());
            telemetry.addData("Right encoder", robot.right.getCurrentPosition());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        // Stop
        robot.update(0, 0);

        sleep(500);

        timeout.reset();
        slide.setTargetPosition((int) (ticksForSlide * 0.6));
        slide.setPower(1);
        while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.addData("R, G, B", "%d, %d, %d",
                    beaconColorSensor.red()*8,
                    beaconColorSensor.green()*8,
                    beaconColorSensor.blue()*8
            );
            telemetry.update();
            idle();
        }
        // Press the button
        slide.setPower(0);

        // If the color sensor senses red, this branch runs
        if (beaconColorSensor.red()*8 > redThreshold || beaconColorSensor.blue()*8 < blueThreshold) {
            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * firstBeaconPushPosition));
            slide.setPower(1);
            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red()*8,
                        beaconColorSensor.green()*8,
                        beaconColorSensor.blue()*8
                );
                telemetry.update();
                idle();
            }
            // Press the button
            slide.setPower(0);
            sleep(1500);
            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * 0.75));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
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
            // Or move on to the other button
            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * 0.75));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
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

            // Drive to the other button
            robot.setMotorTargets(robot.left.getCurrentPosition() - ticksForDistance(firstBeaconButtonDriveDistance), robot.right.getCurrentPosition() - ticksForDistance(firstBeaconButtonDriveDistance));

            robot.update(0.4, 0.4);

            while (!isStopRequested() && robot.areMotorsBusy(true)) {
                telemetry.addData("Left encoder", robot.left.getCurrentPosition());
                telemetry.addData("Right encoder", robot.right.getCurrentPosition());
                //telemetry.addData("Color sensor", beaconColorSensor.argb());
                telemetry.update();
                idle();
            }

            robot.update(0, 0);

            sleep(500);

            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * firstBeaconPushPosition));
            slide.setPower(1);
            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
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
            sleep(1500);
            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * 0.75));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
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

        robot.setMotorTargets(robot.left.getCurrentPosition()+ticksForDistance(backDriveTurnDistance), robot.right.getCurrentPosition()+ticksForDistance(23));
        robot.update(-0.4, -0.4);

        while (!isStopRequested() && robot.areMotorsBusy(true)) {
            telemetry.addData("Left encoder", robot.left.getCurrentPosition());
            telemetry.addData("Right encoder", robot.right.getCurrentPosition());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        // Turn off encoders
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Repeat the steps in lines 233 to 332 for the second beacon
        robot.update(secondTurn, -0.4);

        while (!isStopRequested() && opticalStop.getRawLightDetected() < lightThreshold) {
            telemetry.addData("Light (level red)", lightSensorRed.getLightDetected());
            telemetry.addData("Light (raw red)", lightSensorRed.getRawLightDetected());
            telemetry.addData("Light (level blue)", lightSensorBlue.getLightDetected());
            telemetry.addData("Light (raw blue)", lightSensorBlue.getRawLightDetected());
            telemetry.addData("Threshold", lightThreshold);
            telemetry.addData("Encoder", slide.getCurrentPosition());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Drive forward to the first button
        robot.setMotorTargets(robot.left.getCurrentPosition()-ticksForDistance(secondInitialDrive), robot.right.getCurrentPosition()-ticksForDistance(secondInitialDrive));

        robot.update(0.2, 0.2);

        while (!isStopRequested() && robot.areMotorsBusy(true)) {
            telemetry.addData("Left encoder", robot.left.getCurrentPosition());
            telemetry.addData("Right encoder", robot.right.getCurrentPosition());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);
        timeout.reset();
        slide.setTargetPosition((int) (ticksForSlide * secondBeaconCheckPosition));
        slide.setPower(1);
        while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.addData("R, G, B", "%d, %d, %d",
                    beaconColorSensor.red()*8,
                    beaconColorSensor.green()*8,
                    beaconColorSensor.blue()*8
            );
            telemetry.update();
            idle();
        }
        // Press the button
        slide.setPower(0);

        if (beaconColorSensor.red()*8 > redThreshold && beaconColorSensor.blue()*8 < blueThreshold) {
            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * 0.5));
            slide.setPower(1);
            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
                telemetry.addData("Encoder", slide.getCurrentPosition());
                telemetry.addData("R, G, B", "%d, %d, %d",
                        beaconColorSensor.red()*8,
                        beaconColorSensor.green()*8,
                        beaconColorSensor.blue()*8
                );
                telemetry.update();
                idle();
            }
            // Press the button
            slide.setPower(0);
            sleep(1500);
            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * 0.75));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
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
            // Or move on to the other button
            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * 0.75));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
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

            // Drive to the other button
            robot.setMotorTargets(robot.left.getCurrentPosition() - ticksForDistance(secondBeaconButtonDriveDistance), robot.right.getCurrentPosition() - ticksForDistance(secondBeaconButtonDriveDistance));

            robot.update(0.2, 0.2);

            while (!isStopRequested() && robot.areMotorsBusy(true)) {
                telemetry.addData("Left encoder", robot.left.getCurrentPosition());
                telemetry.addData("Right encoder", robot.right.getCurrentPosition());
                //telemetry.addData("Color sensor", beaconColorSensor.argb());
                telemetry.update();
                idle();
            }

            robot.update(0, 0);

            sleep(500);

            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * 0.5));
            slide.setPower(1);
            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
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
            sleep(1500);
            timeout.reset();
            slide.setTargetPosition((int) (ticksForSlide * 0.75));

            slide.setPower(1);

            while (!isStopRequested() && slide.isBusy() && timeout.seconds() < 2) {
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