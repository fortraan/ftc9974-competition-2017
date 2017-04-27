package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.AppUtil;

/**
 * Created by christopher.johnson on 11/20/16.
 * Put to use on 2/17/17.
 */

@TeleOp(name="Sensor display", group="R/D")
public class Testing extends OpMode {

    //FtcRobotControllerActivity mainActivity;
    private Context context;

    @Override
    public void init() {
        //mainActivity = AppUtil.getInstance().getActivity()
        context = hardwareMap.appContext;
    }

    @Override
    public void loop() {

    }

    private void sendSensorList() {
        Telemetry.Line motorsLine = telemetry.addLine("Configured motors");
        for (DcMotor dcMotor : hardwareMap.dcMotor) {
            //motorsLine.addData(String.format("DcMotor \"%s\"", dcMotor.getDeviceName()), String.format("Connected to "))
        }
    }
}
