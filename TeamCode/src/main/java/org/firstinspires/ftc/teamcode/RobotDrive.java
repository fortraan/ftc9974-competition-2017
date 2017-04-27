package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by christopher.johnson on 10/30/16.
 *
 * Easy robot config!
 *
 * <p>By using object-oriented programming, we can easily create a robot drivetrain system
 * by instantiating this class</p>
 */

@SuppressWarnings("ALL")
public class RobotDrive {

    DcMotor left;
    DcMotor right;

    DcMotor w1;
    DcMotor w2;
    DcMotor w3;

    // Only RANGER has been implemented as of yet.
    // TODO: Implement these

    public enum MODES {
        RANGER,
        TANK,
        KIWI,
        HOLONOMIC,
    }

    // Older code still uses these, so it could be called "legacy" support
    public static final int RANGER = 0;
    public static final int TANK = 1;
    public static final int KIWI = 2;
    public static final int HOLONOMIC = 3;

    DcMotor motors[];

    MODES mode;

    private double curveSensitivity = 0.5;

    /**
     * Construct a new robot config
     * @param mode RANGER, TANK, KIWI, or HOLONOMIC
     * @param hw hardware map
     */
    public RobotDrive(MODES mode, HardwareMap hw) {
        switch (mode) {
            case RANGER:
                left = hw.dcMotor.get("FL");
                left.setDirection(DcMotorSimple.Direction.REVERSE);
                right = hw.dcMotor.get("FR");
                // TODO find a better way to do this
                motors = new DcMotor[] {left, right};
                this.mode = mode;
                break;
            case KIWI:
                w1 = hw.dcMotor.get("W1");
                w2 = hw.dcMotor.get("W2");
                w3 = hw.dcMotor.get("W3");
                this.mode = mode;
                break;
        }
    }

    /**
     * Construct a new robot config, with legacy modes
     * @param mode RANGER, TANK, KIWI, or HOLONOMIC
     * @param hw hardware map
     */
    public RobotDrive(int mode, HardwareMap hw) {
        switch (mode) {
            case RANGER:
                left = hw.dcMotor.get("FL");
                right = hw.dcMotor.get("FR");
                right.setDirection(DcMotorSimple.Direction.REVERSE);
                // TODO find a better way to do this
                motors[0] = left;
                motors[1] = right;
                break;
        }
    }

    /**
     * Set the mode of the drivetrain motors
     * @param mode new motor {@link DcMotor.com.qualcomm.robotcore.hardware.DcMotor.RunMode RunMode}
     */
    public void setMode(DcMotor.RunMode mode) {
        if (this.mode == MODES.RANGER) {
            this.right.setMode(mode);
            this.left.setMode(mode);
        }
    }

    /**
     * Handle driving for the left and right inputs
     * @param left left analog stick
     * @param right right analog stick
     */
    public void update(double left, double right) {
        this.left.setPower(-left);
        this.right.setPower(-right);
    }

    /**
     * Set the encoder targets for the motors
     * @param leftTarget new encoder target
     * @param rightTarget new encoder target
     */
    public void setMotorTargets(int leftTarget, int rightTarget) {
        this.left.setTargetPosition(leftTarget);
        this.right.setTargetPosition(rightTarget);
    }

    /**
     * Returns true if the motors are busy.
     * @param both if both motors must be busy to return true
     * @return if the motors are busy
     */
    public boolean areMotorsBusy(boolean both) {
        if (both) {
            return (right.isBusy() && left.isBusy());
        } else {
            return (right.isBusy() || left.isBusy());
        }
    }

    /**
     * The RobotDrive.Drive command, ported from the C++ WPIlib
     * @param power power of motors
     * @param curve angle to turn
     */
    public void curve(double power, double curve) {
        double leftOut;
        double rightOut;

        if (curve < 0) {
            double curveLog = Math.log(-curve);
            double ratio = (curveLog - curveSensitivity) / (curveLog + curveSensitivity);
            ratio = (ratio == 0) ? 0.0000000001 : ratio;
            leftOut = power / ratio;
            rightOut = power;
        } else if (curve > 0) {
            double curveLog = Math.log(curve);
            double ratio = (curveLog - curveSensitivity) / (curveLog + curveSensitivity);
            ratio = (ratio == 0) ? 0.0000000001 : ratio;
            leftOut = power;
            rightOut = power / ratio;
        } else {
            leftOut = power;
            rightOut = power;
        }

        this.update(leftOut, rightOut);
    }

    /**
     * Like curve(), but it does it in place
     * @param power power of motors
     * @param curve angle to turn
     */
    public void countersteerCurve(double power, double curve) {
        double leftOut;
        double rightOut;

        if (curve < 0) {
            double curveLog = Math.log(-curve);
            double ratio = (curveLog - curveSensitivity) / (curveLog + curveSensitivity);
            ratio = (ratio == 0) ? 0.0000000001 : ratio;
            leftOut = -power / ratio;
            rightOut = power / ratio;
        } else if (curve > 0) {
            double curveLog = Math.log(curve);
            double ratio = (curveLog - curveSensitivity) / (curveLog + curveSensitivity);
            ratio = (ratio == 0) ? 0.0000000001 : ratio;
            leftOut = power / ratio;
            rightOut = -power / ratio;
        } else {
            leftOut = power;
            rightOut = power;
        }

        this.update(leftOut, rightOut);
    }
}
