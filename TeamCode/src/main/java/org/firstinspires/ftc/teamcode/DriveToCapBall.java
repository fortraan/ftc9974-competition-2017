package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by christopher.johnson on 1/16/17.
 * <p>
 * A template for Autos
 * <p>
 * Template instantiated by christopher.johnson
 */

@Autonomous(name = "Drive to cap ball", group = "2017")
public class DriveToCapBall extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        waitForStart();

        launcher.setPower(0.57);

        sleep(4000);

        intake.setPower(-0.7);

        sleep(1500);

        intake.setPower(0);
        launcher.setPower(0);

        sleep(4500);

        robot.update(-1, -1);

        sleep(2500);

        robot.update(1, -1);

        sleep(250);

        robot.update(0, 0);
    }
}
