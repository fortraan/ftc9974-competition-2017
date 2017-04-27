package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by christopher.johnson on 2/17/17.
 */

@TeleOp(name="ODS Diagnostics", group="2017")
public class ODSDiagnostics extends OpMode {

    OpticalDistanceSensor ods;
    private double baseline;
    /*private double averageBaseline;
    private double sumOfReadings;
    private int numReadings;*/

    @Override
    public void init() {
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        /*averageBaseline = 0;
        sumOfReadings = 0;
        numReadings = 0;*/
        baseline = ods.getLightDetected();
    }

    @Override
    public void init_loop() {
        super.init_loop();

        telemetry.addData("Baseline", baseline);
        telemetry.addData("Reading", ods.getLightDetected());
    }

    /*@Override
    protected void postInitLoop() {
        super.postInitLoop();

        sumOfReadings +=
    }*/

    @Override
    public void loop() {
        telemetry.addData("Baseline", baseline);
        telemetry.addData("Reading", ods.getLightDetected());
    }
}
