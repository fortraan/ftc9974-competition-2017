package org.firstinspires.ftc.teamcode;

import android.content.Intent;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RnD.SpeechEngine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

/**
 * Created by christopher.johnson on 1/16/17.
 * <p>
 * A template for Autos
 * <p>
 * Template instantiated by christopher.johnson
 */

@Autonomous(name = "STT test", group = "2017")
public class RecognitionTest extends LinearOpMode {
    private boolean ready = false;
    private boolean speak = false;
    private ArrayList<String> speech;
    private SpeechEngine tts;

    @Override
    public void runOpMode() throws InterruptedException {
        String promptStrings[][] = {
                {"what", "can", "you", "do"},
                {"what", "is", "your", "name"},
                {"hello"}
        };
        Map<String[], Runnable> prompts = Collections.emptyMap();

        SpeechRecognizer recognizer = SpeechRecognizer.createSpeechRecognizer(hardwareMap.appContext);
        tts = new SpeechEngine(hardwareMap.appContext);
        RecognitionListener callback = new RecognitionListener() {
            @Override
            public void onReadyForSpeech(Bundle params) {
                ready = true;
            }

            @Override
            public void onBeginningOfSpeech() {

            }

            @Override
            public void onRmsChanged(float rmsdB) {

            }

            @Override
            public void onBufferReceived(byte[] buffer) {

            }

            @Override
            public void onEndOfSpeech() {

            }

            @Override
            public void onError(int error) {

            }

            @Override
            public void onResults(Bundle results) {
                speech = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
                speak = true;
            }

            @Override
            public void onPartialResults(Bundle partialResults) {

            }

            @Override
            public void onEvent(int eventType, Bundle params) {

            }
        };

        prompts.put(
                new String[]{"what", "can", "you", "do"},
                new Runnable() {
                    @Override
                    public void run() {
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
                    }
                }
        );
        prompts.put(
                new String[]{"what", "is", "your", "name"},
                new Runnable() {
                    @Override
                    public void run() {
                        tts.speak("My name is Pusheen.");
                        while (!isStopRequested() && tts.isSpeaking()) {
                            idle();
                        }
                    }
                }
        );
        prompts.put(
                new String[]{"hello"},
                new Runnable() {
                    @Override
                    public void run() {
                        tts.speak("Hi!");
                        while (!isStopRequested() && tts.isSpeaking()) {
                            idle();
                        }
                    }
                }
        );

        while (!isStopRequested() && !isStarted()) {
            if (tts.doneIniting() && !tts.isSpeaking()) {
                tts.speak("Speech synthesis online.");
            }
        }

        recognizer.setRecognitionListener(callback);
        while (!isStarted() && !isStopRequested()) {
            if (ready && !tts.isSpeaking()) {
                tts.speak("Speech recognition online.");
            }
        }
        recognizer.startListening(new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH));

        while (!isStopRequested()) {
            if (speak && !tts.isSpeaking()) {
                //noinspection SuspiciousMethodCalls
                if (prompts.containsKey(speech.toArray())) {
                    //noinspection SuspiciousMethodCalls
                    prompts.get(speech.toArray()).run();
                }
                speak = false;
            }
        }

        tts.shutdown();
        recognizer.stopListening();
        recognizer.destroy();

        Toast.makeText(hardwareMap.appContext, "Teardown complete", Toast.LENGTH_SHORT).show();
    }
}
