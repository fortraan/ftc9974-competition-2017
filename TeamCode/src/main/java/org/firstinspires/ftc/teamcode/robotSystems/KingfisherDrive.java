package org.firstinspires.ftc.teamcode.robotSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by christopher.johnson on 4/19/17.
 *
 * Drive system for the theoretical "Kingfisher" drive
 *
 * This version assumes a 4-wheel system
 *         wheel1
 *           |
 * wheel2 --   -- wheel3
 *           |
 *         wheel4
 */

public class KingfisherDrive {

    KingfisherWheelPod wheel1;
    KingfisherWheelPod wheel2;
    KingfisherWheelPod wheel3;
    KingfisherWheelPod wheel4;

    public KingfisherDrive(HardwareMap hw) {
        wheel1 = new KingfisherWheelPod(hw, 1);
        wheel2 = new KingfisherWheelPod(hw, 2);
        wheel3 = new KingfisherWheelPod(hw, 3);
        wheel4 = new KingfisherWheelPod(hw, 4);
    }
}
