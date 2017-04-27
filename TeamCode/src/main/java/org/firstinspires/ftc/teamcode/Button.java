package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by christopher.johnson on 2/14/17.
 */

@TeleOp(name = "Button")
public class Button extends OpMode {
    TouchSensor touchSensor;

    @Override
    public void init() {
        touchSensor = hardwareMap.touchSensor.get("touch");
    }

    @Override
    public void loop() {
        telemetry.addData("Button", touchSensor.isPressed());
    }
}
