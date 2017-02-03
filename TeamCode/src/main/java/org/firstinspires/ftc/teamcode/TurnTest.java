package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by christopher.johnson on 1/16/17.
 * <p>
 * A template for Autos                                                                 b
 * <p>
 * Template instantiated by christopher.johnson
 */

@Autonomous(name = "Turn Test", group = "2017")
public class TurnTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final int target = -270;

        RobotDrive robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating()) {
            idle();
        }

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initDif = gyro.getIntegratedZValue() + target;

        robot.update(-0.16, 0.16);

        while (!isStopRequested() && gyro.getIntegratedZValue() < -target) {

            double drive = 0.75 * Math.abs(gyro.getIntegratedZValue() + target) / initDif;
            drive = (drive < 0.12) ? 0.12 : drive;
            drive = (drive > 0.75) ? 0.75 : drive;

            robot.countersteerCurve(drive, 1);

            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.addData("Integrated Heading", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        robot.update(0, 0);
    }
}
