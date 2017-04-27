package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by christopher.johnson on 10/30/16.
 */
@TeleOp(name="Competition", group="2017")
public class CompetitionTeleOp extends OpMode {

    RobotDrive robot;

    DcMotor ll;
    DcMotor rl;
    DcMotor intake;
    DcMotor slide;
    DcMotor lift;

    //Servo liftRelease;

    ElapsedTime buttonHeldDetector;
    boolean buttonHeldDetectorStarted = false;

    ElapsedTime liftRetract;
    boolean liftTriggered = false;

    boolean controlOverride = false;
    Gamepad gamepadOneLastUpdate;
    Gamepad gamepadTwoLastUpdate;

    //PWMOutput liftRelease;
    //ServoController releaseController;
    Servo servo;

    @Override
    public void init() {
        robot = new RobotDrive(RobotDrive.MODES.RANGER, hardwareMap);
        ll = hardwareMap.dcMotor.get("ll");
        rl = hardwareMap.dcMotor.get("rl");
        ll.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake = hardwareMap.dcMotor.get("intake");
        slide = hardwareMap.dcMotor.get("slide");
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///liftRelease = hardwareMap.servo.get("release");
        buttonHeldDetector = new ElapsedTime();
        liftRetract = new ElapsedTime();
        gamepadOneLastUpdate = gamepad1;
        gamepadTwoLastUpdate = gamepad2;

        //liftRelease = hardwareMap.pwmOutput.get("release");
        //liftRelease.setPulseWidthPeriod(1500);

        //releaseController = hardwareMap.servoController.get("Servo Controller 1");
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void init_loop() {
        super.init_loop();

        //telemetry.addData("Servo position", liftRelease.getPulseWidthPeriod());
        //telemetry.addData("Servo two", releaseController.setServoPosition();)
        telemetry.addData("Servo", servo.getPosition());
        gamepadOneLastUpdate = gamepad1;
        gamepadTwoLastUpdate = gamepad2;
    }

    @Override
    public void loop() {
        if (gamepad1.back && !gamepadOneLastUpdate.back) {
            controlOverride = !controlOverride;
        }

        if (//(gamepad1.left_bumper || lift.getCurrentPosition() > 300) && !gamepad1.right_bumper) {
                gamepad1.dpad_down) {
            robot.update(-gamepad1.right_stick_y * 0.2, -gamepad1.right_stick_y * 0.2);
        } /*else if (gamepad1.dpad_up) {
            robot.update();
        }*/ else {
            robot.update(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }
        if (gamepad2.left_bumper || gamepad2.right_bumper) {

            // MOTOR LAUNCH SPEED

            ll.setPower(0.57);
            rl.setPower(-0.57);
        }  else if (gamepad2.dpad_down) {
            ll.setPower(-0.4);
            rl.setPower(0.4);
        } else if (gamepad2.right_trigger > 0) {
            ll.setPower(gamepad2.right_trigger);
            rl.setPower(-gamepad2.right_trigger);
        } else {
            ll.setPower(0);
            rl.setPower(0);
        }
        if (gamepad1.dpad_left && controlOverride) {
            slide.setPower(-0.4);
        } /*else if (gamepad1.dpad_right && controlOverride) {
            slide.setPower(0.4);
        }*/ else {
            slide.setPower(0);
        }
        if ((gamepad2.x || (controlOverride && gamepad1.x)) && lift.getCurrentPosition() >= 0 && lift.getCurrentPosition() <= 34500) {
            lift.setPower(-0.3);
        } else if ((gamepad2.y || (controlOverride && gamepad1.y)) && lift.getCurrentPosition() >= 0 && lift.getCurrentPosition() <= 34500) {
            lift.setPower(1);
        } else if (lift.getCurrentPosition() < 0){
            lift.setPower(1);
        } else if (lift.getCurrentPosition() > 34500) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }
        telemetry.addData("Encoder", lift.getCurrentPosition());
        if (gamepad2.a) {
            intake.setPower(-1);
        } else if (gamepad2.b) {
            intake.setPower(0.7);
        } else if (gamepad2.left_trigger > 0) {
            intake.setPower(-gamepad2.left_trigger);
        } else {
            intake.setPower(0);
        }

        if (gamepad2.dpad_up && !buttonHeldDetectorStarted) {
            buttonHeldDetector.reset();
            buttonHeldDetectorStarted = true;
        }
        if (!gamepad2.dpad_up && buttonHeldDetectorStarted) {
            buttonHeldDetectorStarted = false;
        }
        if (buttonHeldDetector.seconds() >= 2 && buttonHeldDetectorStarted) {
            //liftRelease.setPulseWidthPeriod(1550);
            servo.setPosition(0.03);
            liftRetract.reset();
            liftTriggered = true;
            buttonHeldDetectorStarted = false;
        }
        if (liftTriggered && liftRetract.seconds() >= 2) {
            //liftRelease.setPulseWidthPeriod(1500);
            servo.setPosition(0);
            liftTriggered = false;
        }
        if (controlOverride) {
            telemetry.addLine("Override active");
        }

        telemetry.addData("Start", gamepad2.dpad_up && !buttonHeldDetectorStarted);
        telemetry.addData("Stop", !gamepad2.dpad_up && buttonHeldDetectorStarted);
        telemetry.addData("Button hold", buttonHeldDetector.seconds());
        telemetry.addData("Release", buttonHeldDetector.seconds() >= 2 && buttonHeldDetectorStarted);
        telemetry.addData("Triggered", liftTriggered);
        telemetry.addData("Started", buttonHeldDetectorStarted);

        /*int buttons = 0;
        try {
            buttons = parseByteBuffer(ByteBuffer.wrap(gamepad2.toByteArray()));
        } catch (RobotCoreException e) {
            RobotLog.e(e.getMessage());
        }
        buttons &*/
        //intake.setPower();
        /*ByteBuffer buffer = null;
        try {
            buffer = parseByteBuffer(ByteBuffer.wrap(gamepad2.toByteArray()));
        } catch (RobotCoreException e) {
            RobotLog.e(e.getMessage());
        }
        if (buffer != null) {
            telemetry.addData("Left trigger", buffer.getFloat());
            telemetry.addData("Right trigger", buffer.getFloat());

            int buttons = buffer.getInt();
            telemetry.addData("Dpad down", buttons & 0x00800);
            telemetry.addData("A", buttons & 0x00100);
            telemetry.addData("B", buttons & 0x00080);
            telemetry.addData("Left bumper", buttons & 0x00002);
            telemetry.addData("Right bumper", buttons & 0x00001);
        }*/
        telemetry.addData("Speed", ll.getPower());

        gamepadOneLastUpdate = gamepad1;
        gamepadTwoLastUpdate = gamepad2;
    }

    /*private ByteBuffer parseByteBuffer(ByteBuffer byteBuffer) {
        byteBuffer.getInt();
        byteBuffer.getLong();
        byteBuffer.getFloat();
        byteBuffer.getFloat();
        byteBuffer.getFloat();
        byteBuffer.getFloat();

        return byteBuffer;
    }*/
}
