package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by christopher.johnson on 10/30/16.
 */
@TeleOp(name="Test", group="2017")

public class Test extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor ll;
    DcMotor rl;
    DcMotor lift;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("FL");
        right = hardwareMap.dcMotor.get("FR");
        ll = hardwareMap.dcMotor.get("ll");
        rl = hardwareMap.dcMotor.get("rl");
        ll.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if ((gamepad1.left_bumper || lift.getCurrentPosition() > 300) && !gamepad1.right_bumper) {
            left.setPower(-gamepad1.right_stick_y*0.2);
            right.setPower(gamepad1.right_stick_y*0.2);
        } else {
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(gamepad1.right_stick_y);
        }
        if (gamepad1.a) {
            ll.setPower(1);
            rl.setPower(-1);
        } else if (gamepad1.b) {
            ll.setPower(-1);
            rl.setPower(1);
        } else {
            ll.setPower(0);
            rl.setPower(0);
        }
        if (gamepad1.x && lift.getCurrentPosition() >= 0 && lift.getCurrentPosition() <= 34000) {
            lift.setPower(-0.3);
        } else if (gamepad1.y && lift.getCurrentPosition() >= 0 && lift.getCurrentPosition() <= 34000) {
            lift.setPower(1);
        } else if (lift.getCurrentPosition() < 0){
            lift.setPower(1);
        } else if (lift.getCurrentPosition() > 34000) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }
        telemetry.addData("Encoder", lift.getCurrentPosition());
    }
}
