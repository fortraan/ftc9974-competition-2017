package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by christopher.johnson on 2/1/17.
 */

@TeleOp(name="Lift")
public class LiftDiag extends OpMode {
    DcMotor lift;
    @Override
    public void init() {
        lift = hardwareMap.dcMotor.get("lift");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            lift.setPower(-0.3);
        } else if (gamepad1.dpad_up) {
            lift.setPower(1);
        } else {
            lift.setPower(0);
        }
    }
}
