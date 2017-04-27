package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareMap.DeviceMapping;

@Autonomous(name="Just Fling", group="2017")
public class JustFling extends LinearOpMode
{
    public void runOpMode()
            throws InterruptedException
    {
        Launcher launcher = new Launcher(this.hardwareMap);
        DcMotor intake = (DcMotor)this.hardwareMap.dcMotor.get("intake");

        waitForStart();

        launcher.setPower(0.57D);

        sleep(4000L);

        intake.setPower(-0.7D);

        sleep(1500L);

        intake.setPower(0.0D);
        launcher.setPower(0.0D);
    }
}