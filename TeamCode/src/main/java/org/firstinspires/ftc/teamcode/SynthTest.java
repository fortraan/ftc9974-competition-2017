package org.firstinspires.ftc.teamcode;

import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RnD.SpeechEngine;

/**
 * Created by christopher.johnson on 1/16/17.
 * <p>
 * A template for Autos
 * <p>
 * Template instantiated by christopher.johnson
 */

@Autonomous(name = "Speech test", group = "2017")
public class SynthTest extends LinearOpMode {

    SpeechEngine tts;
    Launcher launcher;
    DcMotor slide;
    TouchSensor stop;
    DcMotor lift;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            tts = new SpeechEngine(hardwareMap.appContext);
        } catch (InstantiationError e) {
            Toast.makeText(hardwareMap.appContext, e.getMessage(), Toast.LENGTH_SHORT).show();
            requestOpModeStop();
        }
        launcher = new Launcher(hardwareMap);
        slide = hardwareMap.dcMotor.get("slide");
        stop = hardwareMap.touchSensor.get("stop");
        lift = hardwareMap.dcMotor.get("lift");

        while (!isStopRequested() && !isStarted()) {
            if (tts.doneIniting()) {
                telemetry.addLine("TTS has finished initing.");
            }
        }

        while (!isStopRequested() && !tts.doneIniting()) {
            idle();
        }

        tts.speak("Hello.");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        tts.speak("I am Pusheen.");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        tts.speak("I can do things such as.");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        tts.speak("Fire particles.");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        launcher.setPower(0.57);
        sleep(1500);
        launcher.setPower(0);

        sleep(3000);

        tts.speak("Push beacons in autonomous.");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        if (!stop.isPressed()) {
            slide.setPower(-0.2);
        }
        sleep(400);
        slide.setPower(0.2);
        sleep(400);
        slide.setPower(0);

        tts.speak("And lift the cap ball.");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        lift.setPower(1);
        sleep(1000);
        lift.setPower(0);

        tts.speak("Thank you for volunteering your time today. What questions can we answer for you?");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        tts.shutdown();
    }
}
