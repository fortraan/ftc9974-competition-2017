package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by christopher.johnson on 1/16/17.
 *
 * A basic program to test the encoders.
 */

@Autonomous(name="Encoder Test", group="2017")
public class EncoderAuto extends LinearOpMode {
    DcMotor left;
    DcMotor right;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.dcMotor.get("FL");
        right = hardwareMap.dcMotor.get("FR");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);
        idle();

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setTargetPosition(1270);
        right.setTargetPosition(1270);

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Left encoder", left.getCurrentPosition());
            telemetry.addData("Right encoder", right.getCurrentPosition());
            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;

        left.setPower(1);
        right.setPower(1);

        while (!isStopRequested() && (left.isBusy() || right.isBusy())) {
            telemetry.addData("Left encoder", left.getCurrentPosition());
            telemetry.addData("Right encoder", right.getCurrentPosition());
            telemetry.update();
            idle();
        }

        left.setPower(0);
        right.setPower(0);
    }
}
