package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
/**
 * Created by christopher.johnson on 1/16/17.
 * <p>
 * A template for Autos
 * <p>
 * Template instantiated by christopher.johnson
 */

@Autonomous(name = "SlideTest", group = "2017")
public class SlideTest extends LinearOpMode {
    // Number of ticks needed to go all the way across the slide
    final int ticksForSlide = 5340;
    final int redThreshold = 32;
    final int blueThreshold = 16;

    DcMotor slide;
    TouchSensor stop;

    private int ticksForDistance(double distance) {
        return (int) Math.round((distance * 1680) / (4*Math.PI));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);
        slide = hardwareMap.dcMotor.get("slide");
        stop = hardwareMap.touchSensor.get("stop");

        ModernRoboticsI2cColorSensor beaconColorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("beaconRed");
        //beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x22));
        //beaconColorSensor.enableLed(true);

        float hsvVals[] = {0,0,0};

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            Color.RGBToHSV(beaconColorSensor.red() * 8, beaconColorSensor.green() * 8, beaconColorSensor.blue() * 8, hsvVals);
            //telemetry.addData("H, S, V", "%d, %d, %d", hsvVals[0], hsvVals[1], hsvVals[2]);
            //telemetry.addData("Color", beaconColorSensor.argb());
            telemetry.addData("R, G, B", "%d, %d, %d",
                    beaconColorSensor.red()*8,
                    beaconColorSensor.green()*8,
                    beaconColorSensor.blue()*8
            );
            telemetry.update();
            idle();
        }

        //beaconColorSensor.enableLed(false);

        slide.setPower(-0.2);
        while (!stop.isPressed() && !isStopRequested()) {
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

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.setTargetPosition(ticksForSlide/2);
        slide.setPower(0.3);

        while (slide.isBusy() && !isStopRequested()) {
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

        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMotorTargets(robot.left.getCurrentPosition()-ticksForDistance(11.5), robot.right.getCurrentPosition()-ticksForDistance(11.5));

        robot.update(0.5, 0.5);

        while (!isStopRequested() && robot.areMotorsBusy(true)) {
            telemetry.addData("Left encoder", robot.left.getCurrentPosition());
            telemetry.addData("Right encoder", robot.right.getCurrentPosition());
            //telemetry.addData("Color sensor", beaconColorSensor.argb());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);

        sleep(1000);

        slide.setTargetPosition((int) (ticksForSlide * 0.15));

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

        sleep(1000);

        /*if (beaconColorSensor.red()*8 > redThreshold && beaconColorSensor.blue()*8 < blueThreshold) {
            slide.setTargetPosition(0);
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
        }
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

        slide.setPower(0);*/
        if (beaconColorSensor.red()*8 > redThreshold && beaconColorSensor.blue()*8 < blueThreshold) {
            slide.setTargetPosition(0);
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

            robot.setMotorTargets(robot.left.getCurrentPosition() - ticksForDistance(7), robot.right.getCurrentPosition() - ticksForDistance(7));

            robot.update(0.5, 0.5);

            while (!isStopRequested() && robot.areMotorsBusy(true)) {
                telemetry.addData("Left encoder", robot.left.getCurrentPosition());
                telemetry.addData("Right encoder", robot.right.getCurrentPosition());
                //telemetry.addData("Color sensor", beaconColorSensor.argb());
                telemetry.update();
                idle();
            }

            robot.update(0, 0);

            sleep(500);

            slide.setTargetPosition(0);
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
    }
}
