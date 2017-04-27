package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by christopher.johnson on 4/10/17.
 *
 * "Added in" format: Added in V[version]p[patch]
 *
 * <h2>Version 1, patch 0</h2>
 * Date: 4/12/17
 * Observations:
 * Wheel 1 skids when strafing left or right, resulting in the robot turning.
 *
 * TODO <h2>Version 2, patch 0</h2>
 * Changelog:
 * Added gyroscopic turn correction
 */

@SuppressWarnings("WeakerAccess")
public class KiwiDrive {

    // Added in V2p0
    ModernRoboticsI2cGyro gyro;
    double error;
    double movementMag;
    double movementTheta;
    double correction;
    double integratedTheta;
    double integratedX;
    double integratedY;
    double correctedTheta;
    double correctedX;
    double correctedY;
    double target;
    double lastRot;

    PID correctionPid;

    // Added in V1p0
    DcMotor wheel1;
    DcMotor wheel2;
    DcMotor wheel3;

    public KiwiDrive(HardwareMap hw) {
        wheel1 = hw.dcMotor.get("W1");
        wheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel2 = hw.dcMotor.get("W2");
        wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel3 = hw.dcMotor.get("W3");
        wheel3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = (ModernRoboticsI2cGyro) hw.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        gyro.calibrate();

        correctionPid = new PID(-10000, 10000, target, 0.00001, 10, 1, 0);
    }

    public void move(double x, double y, double rot, boolean correct, boolean applyPid) {

        movementMag = Math.sqrt((x * x) + (y * y));
        if (movementMag > 0.1) {
            movementTheta = Math.toDegrees(Math.atan2(y, x));// + 90;
        } else {
            movementTheta = 0;
        }

        error = target - gyro.getIntegratedZValue();
        correctionPid.setInput(gyro.getIntegratedZValue());
        if (correctionPid.DoCycle()) {
            correction = correctionPid.output;
        }

        integratedTheta = (gyro.getIntegratedZValue() - movementTheta);


        //
        integratedX = movementMag * Math.cos(Math.toRadians(integratedTheta));
        integratedY = movementMag * Math.sin(Math.toRadians(integratedTheta));



        //if (correct) {
        correctedTheta = (movementTheta) - gyro.getIntegratedZValue();
        //}
        correctedX = movementMag * Math.cos(Math.toRadians(correctedTheta));
        correctedY = movementMag * Math.sin(Math.toRadians(correctedTheta));

        // Unit Circles
        // Okay, so the Make magazine article says sqrt(3/2), but the arduino code says sqrt(3) / 2
        // wheel1 -> 180 degreees
        // cos(180), sin(180) = (-1, 0)
        // wheel1 = -1x, 0y
        // wheel2 -> -60 degrees/300 degrees
        // cos(-60), sin(-60) = (0.5, -sqrt(3) / 2)
        // wheel2 = 0.5x - (sqrt(3) / 2)y
        // wheel3 = 60 degrees
        // cos(60), sin(60) = (0.5, sqrt(3) / 2)
        // wheel3 = 0.5x + (sqrt(3) / 2)y
        /*
           if there was a wheel at 0 degrees, wheel4 = (cos(0)x, sin(0)y) = (1x, 0y)
         */
        /*
           the Kingfisher would work like this:
           wheel = cos(angle) * x + sin(angle) * y
           angle adjustment:
           angle = slerp(angle, desiredMovementAngle, 0.5)
         */

        if (rot == 0 && lastRot != 0) {
            target = gyro.getIntegratedZValue();
            correctionPid.setSetpoint(target);
        } else if (lastRot == 0 && rot != 0) {
            //
        }

        if (applyPid) {
            if (correct) {
                wheel1.setPower(-correctedX + rot);
                wheel2.setPower((0.5 * correctedX) - (0.866 * correctedY) + rot);
                wheel3.setPower((0.5 * correctedX) + (0.866 * correctedY) + rot);
            } else {
                wheel1.setPower(-x + correction);
                wheel2.setPower(Range.clip((0.5 * x) - (0.866 * y) + correction, -1, 1));
                wheel3.setPower(Range.clip((0.5 * x) + (0.866 * y) + correction, -1, 1));
            }
        } else if (correct) {
            wheel1.setPower(-correctedX + rot);
            wheel2.setPower((0.5 * correctedX) - (0.866 * correctedY) + rot);
            wheel3.setPower((0.5 * correctedX) + (0.866 * correctedY) + rot);
        } else {
            wheel1.setPower(-x + rot);
            wheel2.setPower(Range.clip((0.5 * x) - (0.866 * y) + rot, -1, 1));
            wheel3.setPower(Range.clip((0.5 * x) + (0.866 * y) + rot, -1, 1));
        }

        lastRot = rot;
    }

    public boolean isReady() {
        return !gyro.isCalibrating();
    }

    public void resetForward() {
        gyro.resetZAxisIntegrator();
    }
}
