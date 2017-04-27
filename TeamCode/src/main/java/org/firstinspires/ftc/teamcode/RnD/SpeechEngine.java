package org.firstinspires.ftc.teamcode.RnD;

import android.content.Context;
import android.os.Build;
import android.speech.tts.TextToSpeech;
import android.widget.Toast;

/**
 * Created by christopher.johnson on 2/24/17.
 */

public class SpeechEngine {
    private TextToSpeech tts;
    private boolean ttsReady = false;
    private Context c;

    public SpeechEngine (Context context) throws InstantiationError {
        TextToSpeech.OnInitListener ready = new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
                switch (status) {
                    case TextToSpeech.ERROR:
                        throw new InstantiationError("TTS failed to init");
                    case TextToSpeech.SUCCESS:
                        ttsReady = true;
                        break;
                }
            }
        };
        tts = new TextToSpeech(context, ready);
        c = context;
    }

    public boolean doneIniting () {
        return ttsReady;
    }

    public void speak(String text) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            tts.speak(text, TextToSpeech.QUEUE_ADD, null, "hello");
        } else {
            Toast.makeText(c, "API Level too low", Toast.LENGTH_SHORT).show();
        }
    }

    public boolean isSpeaking () {
        return tts.isSpeaking();
    }

    public void shutdown() {
        tts.shutdown();
    }
}
