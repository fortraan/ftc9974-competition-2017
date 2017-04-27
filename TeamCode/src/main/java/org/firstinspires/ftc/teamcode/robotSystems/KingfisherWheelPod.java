package org.firstinspires.ftc.teamcode.robotSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by christopher.johnson on 4/19/17.
 */

public class KingfisherWheelPod {

    DcMotor wheel;
    DcMotor pivot;
    TouchSensor stop1;
    TouchSensor stop2;

    int wheelNum;

    public KingfisherWheelPod(HardwareMap hw, int wheelNumber) {
        wheelNum = wheelNumber;
        wheel = hw.dcMotor.get("w" + wheelNum);
        pivot = hw.dcMotor.get("wp" + wheelNum);
        stop1 = hw.touchSensor.get("w" + wheelNum + "s1");
        stop2 = hw.touchSensor.get("w" + wheelNum + "s2");
    }
}
