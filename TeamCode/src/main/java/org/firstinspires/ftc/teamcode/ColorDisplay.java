package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/**
 * Created by christopher.johnson on 11/6/16.
 */
@TeleOp(name="Display Color", group="2017")
public class ColorDisplay extends OpMode {
    private DeviceInterfaceModule DIM;
    private ColorSensor cs;

    @Override
    public void init() {
        DIM = hardwareMap.deviceInterfaceModule.get("DIM");
        cs = hardwareMap.colorSensor.get("line");
        cs.enableLed(true);
    }

    @Override
    public void loop() {
        if (cs.argb() == 0) {           // BLACK
            telemetry.addLine("Black");
            DIM.setLED(0, false);
            //DIM.setLED(, false);
        } else if (cs.argb() == 16) {   // WHITE
            telemetry.addLine("White");
            //DIM.setLED(0x01, true);
            //DIM.setLED(0x02, true);
        } else if (cs.argb() == 10) {   // RED
            //DIM.setLED(0x01, true);
            //DIM.setLED(0x02, false);
        } else if (cs.argb() == 3) {    // BLUE
            //DIM.setLED(0x01, false);
            //DIM.setLED(0x02, true);
        }
    }
}
