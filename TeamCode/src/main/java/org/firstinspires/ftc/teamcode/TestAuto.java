package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by christopher.johnson on 10/30/16.
 *
 * Simple auto
 */
@Autonomous(name="Test Auto", group="2017")
@SuppressWarnings("unused")
public class TestAuto extends LinearOpMode {
    private double inches = 53;

    // 420 pulses
    // 1680 counts

    private double generateTarget(double revs) {
        return (Math.PI * 4) * revs;
    }

    private int generateInchTarget(double inches) {
        return (int) (inches / (Math.PI * 4)) * 1680;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left = hardwareMap.dcMotor.get("FL");
        DcMotor right = hardwareMap.dcMotor.get("FR");

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int target = generateInchTarget(inches);

        left.setTargetPosition(target);
        right.setTargetPosition(target);
    }
}
