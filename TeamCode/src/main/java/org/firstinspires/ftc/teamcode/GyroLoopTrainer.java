package org.firstinspires.ftc.teamcode;

import android.widget.Toast;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by christopher.johnson on 1/29/17.
 */

@Autonomous(name="Gyro Loop Trainer")
public class GyroLoopTrainer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);
        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        gyro.calibrate();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Gyro is currently", (gyro.isCalibrating()) ? "calibrating" : "calibrated");
            telemetry.update();
            idle();
        }

        ElapsedTime timer = new ElapsedTime();
        String animation[] = {"", ".", "..", "..."};
        while (!isStopRequested() && gyro.isCalibrating()) {
            telemetry.addLine("Please wait, calibrating" + animation[(int) (timer.seconds()) % 3]);
            telemetry.update();
            idle();
        }

        double Kp = 0.01;
        Map<Double, Double> timePMap = new HashMap<>();

        while (!isStopRequested() && timePMap.size() < 50) {
            gyro.calibrate();
            while (!isStopRequested() && gyro.isCalibrating()) {
                telemetry.addLine("Please wait, calibrating" + animation[(int) (timer.seconds()) % 3]);
                telemetry.update();
                idle();
            }

            timer.reset();

            while (!isStopRequested() && !(gyro.getIntegratedZValue() > 44 && gyro.getIntegratedZValue() < 46)) {
                robot.countersteerCurve(0.16, (gyro.getIntegratedZValue() - 45)*Kp);
            }

            timePMap.put(timer.milliseconds(), Kp);

            Kp += 0.01;
        }

        double lowestTime = Double.MAX_VALUE;
        for (Double time : timePMap.keySet()) {
            if (time < lowestTime) {
                lowestTime = time;
            }
        }

        // In case of the rare event of a tie
        if (lowestTime == Double.MAX_VALUE) {
            Toast.makeText(hardwareMap.appContext, "There was a tie", Toast.LENGTH_LONG).show();
        } else {
            Toast.makeText(hardwareMap.appContext, String.valueOf(timePMap.get(lowestTime)), Toast.LENGTH_LONG).show();
        }
    }
}
