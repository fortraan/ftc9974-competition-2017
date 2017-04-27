package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by christopher.johnson on 4/10/17.
 */

@TeleOp(name = "Kiwi", group = "2017")
public class KiwiTeleOp extends OpMode {

    boolean lastA = false, lastB = false, lastX = false, editing = false;
    double p, i, d;

    private KiwiDrive kiwi;

    @Override
    public void init() {
        kiwi = new KiwiDrive(hardwareMap);
        editing = false;
    }

    @Override
    public void loop() {
        if (!kiwi.isReady()) {
            telemetry.addLine("Calibrating");
            return;
        }
        /*if (gamepad1.a && !lastA && !editing) {
            editing = true;
            p = gamepad1.left_stick_y;
            telemetry.addData("P", p);
        } else if (!gamepad1.a && lastA && editing) {
            editing = false;
            kiwi.correctionPid.setTunings(p, kiwi.correctionPid.I, kiwi.correctionPid.D);
        }

        if (gamepad1.b && !lastB && !editing) {
            editing = true;
            i = gamepad1.left_stick_y;
            telemetry.addData("I", i);
        } else if (!gamepad1.b && lastB && editing) {
            editing = false;
            kiwi.correctionPid.setTunings(kiwi.correctionPid.P, i, kiwi.correctionPid.D);
        }

        if (gamepad1.x && !lastX && !editing) {
            editing = true;
            d = gamepad1.left_stick_y;
            telemetry.addData("D", d);
        } else if (!gamepad1.x && lastX && editing) {
            editing = false;
            kiwi.correctionPid.setTunings(kiwi.correctionPid.P, kiwi.correctionPid.I, d);
        }*/

        kiwi.move(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.left_bumper, gamepad1.right_bumper);
        telemetry.addData("Magnitude", kiwi.movementMag);
        telemetry.addData("Theta", kiwi.movementTheta);
        telemetry.addData("Gyro heading", kiwi.gyro.getHeading());
        telemetry.addData("Error", kiwi.error);
        telemetry.addData("Correction", kiwi.correction);
        telemetry.addData("Integrated Theta", kiwi.integratedTheta);
        telemetry.addData("Integrated X", kiwi.integratedX);
        telemetry.addData("Integrated Y", kiwi.integratedY);
        telemetry.addData("Integrated heading", kiwi.gyro.getIntegratedZValue());
        telemetry.addData("Corrected theta", kiwi.correctedTheta);
        telemetry.addData("Target", kiwi.target);
        telemetry.addData("Hunch", (kiwi.movementTheta - 90) - kiwi.gyro.getIntegratedZValue());
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
    }
}
