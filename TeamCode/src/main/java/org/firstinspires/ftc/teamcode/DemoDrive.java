package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by christopher.johnson on 2/23/17.
 */

@TeleOp(name="Outreach mode")
public class DemoDrive extends OpMode {
    private  RobotDrive robot;

    @Override
    public void init() {
        robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);
    }

    @Override
    public void loop() {
        robot.curve(gamepad1.right_trigger, gamepad1.left_stick_x * -45);
    }
}
