package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "ColorTest", group = "2017")
public class ColorTest extends LinearOpMode {
    // Number of ticks needed to go all the way across the slide
    final int ticksForSlide = 5340;

    DcMotor slide;
    TouchSensor stop;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.dcMotor.get("slide");
        stop = hardwareMap.touchSensor.get("stop");

        ModernRoboticsI2cColorSensor beaconColorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("beaconRed");
        //beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x22));
        beaconColorSensor.enableLed(true);

        slide.setPower(-0.08);
        while (!stop.isPressed() && !isStopRequested()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
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
            telemetry.update();
            idle();
        }

        slide.setPower(0);

        waitForStart();

        if (beaconColorSensor.red() * 8 > 200) {
            slide.setTargetPosition(0);
            slide.setPower(0.1);
        }

        while (slide.isBusy() && !isStopRequested()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.update();
            idle();
        }

        slide.setPower(0);
        sleep(3000);
        slide.setTargetPosition(ticksForSlide/2);
        slide.setPower(0.1);

        while (slide.isBusy() && !isStopRequested()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.update();
            idle();
        }

        slide.setPower(0);
    }
}
