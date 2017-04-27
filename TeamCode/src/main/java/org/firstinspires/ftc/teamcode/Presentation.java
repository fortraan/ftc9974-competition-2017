package org.firstinspires.ftc.teamcode;

import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.RnD.SpeechEngine;

/**
 * Created by christopher.johnson on 1/16/17.
 * <p>
 * A template for Autos
 * <p>
 * Template instantiated by christopher.johnson
 */

@Autonomous(name = "Speech", group = "2017")
public class Presentation extends LinearOpMode {

    SpeechEngine tts;
    //FtcRobotControllerActivity activity;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            tts = new SpeechEngine(hardwareMap.appContext);
        } catch (InstantiationError e) {
            Toast.makeText(hardwareMap.appContext, e.getMessage(), Toast.LENGTH_SHORT).show();
            requestOpModeStop();
        }

        while (!isStopRequested() && !tts.doneIniting()) {
            telemetry.addLine("TTS starting up");
            telemetry.update();
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

        tts.speak("Push beacons in autonomous.");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        tts.speak("And lift the cap ball.");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        tts.speak("Thank you for volunteering your time today. What questions can we answer for you?");
        while (!isStopRequested() && tts.isSpeaking()) {
            idle();
        }

        tts.shutdown();
    }
}
