package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by christopher.johnson on 2/24/17.
 */

public class Launcher {
    private DcMotor ll;
    private DcMotor rl;

    public Launcher (HardwareMap hw) {
        ll = hw.dcMotor.get("ll");
        rl = hw.dcMotor.get("rl");
        ll.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        ll.setPower(power);
        rl.setPower(-power);
    }
}
