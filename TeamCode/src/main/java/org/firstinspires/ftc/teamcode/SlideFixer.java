package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by christopher.johnson on 2/22/17.
 *
 * Slide recentering
 */

@SuppressWarnings("unused")
@Autonomous(name="Slide fixer")
public class SlideFixer extends LinearOpMode {
    @SuppressWarnings("FieldCanBeLocal")
    private final int ticksForSlide = 5340;

    DcMotor slide;
    TouchSensor stop;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.dcMotor.get("slide");
        stop = hardwareMap.touchSensor.get("stop");

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //beaconColorSensor.enableLed(false);

        slide.setPower(-0.);
        while (!stop.isPressed() && !isStopRequested()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.update();
            idle();
        }
        slide.setPower(0);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.setTargetPosition(ticksForSlide / 2);
        slide.setPower(0.3);

        while (slide.isBusy() && !isStopRequested()) {
            telemetry.addData("Encoder", slide.getCurrentPosition());
            telemetry.update();
            idle();
        }

        slide.setPower(0);
    }
}
