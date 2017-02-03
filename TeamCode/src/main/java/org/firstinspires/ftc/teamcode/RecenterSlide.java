package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by christopher.johnson on 1/29/17.
 *
 * The slide on the robot kept breaking, so out of frustration I wrote this.
 * And by breaking, I mean the physical parts breaking, not just the slide
 * coming off the track.
 */

@TeleOp(name="Recenter Slide", group="2017")
public class RecenterSlide extends OpMode {
    private DcMotor slide;

    @Override
    public void init() {
        slide = hardwareMap.dcMotor.get("slide");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
            slide.setPower(0.1);
        } else if (gamepad1.dpad_right) {
            slide.setPower(-0.1);
        } else {
            slide.setPower(0);
        }
    }
}
