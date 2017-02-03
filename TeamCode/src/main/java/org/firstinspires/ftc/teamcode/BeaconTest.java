package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by christopher.johnson on 1/16/17.
 * <p>
 * A template for Autos
 * <p>
 * Template instantiated by christopher.johnson
 */

@Autonomous(name="BeaconTest", group="2017")
public class BeaconTest extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDrive robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);

        DcMotor slide = hardwareMap.dcMotor.get("slide");

        ModernRoboticsI2cColorSensor colorRed = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("red");
        ModernRoboticsI2cColorSensor colorBlue = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("blue");
        colorRed.enableLed(false);
        colorBlue.enableLed(false);

        int startingPos;
        TouchSensor stop1;
        TouchSensor stop2;
        if (!hardwareMap.getAll(TouchSensor.class).isEmpty()) {
            stop1 = hardwareMap.touchSensor.get("stop1");
            stop2 = hardwareMap.touchSensor.get("stop2");

            startingPos = slide.getCurrentPosition();

            slide.setPower(-0.2);
            while (!isStopRequested() && !stop1.isPressed() && !stop2.isPressed()) {
                telemetry.addData("Starting encoder value", startingPos);
                telemetry.addData("Slide encoder value", slide.getCurrentPosition());
                telemetry.update();
            }

            if (isStopRequested()) return;

            if (stop1.isPressed()) {

            }
        }
    }
}
