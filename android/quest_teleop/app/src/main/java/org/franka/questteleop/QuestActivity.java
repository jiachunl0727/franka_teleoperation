package org.franka.questteleop;

import android.app.NativeActivity;
import android.os.Bundle;
import android.speech.tts.TextToSpeech;
import android.util.Log;

import java.util.ArrayDeque;
import java.util.Locale;

public final class QuestActivity extends NativeActivity {
    private static final String TAG = "FrankaQuestTeleop";

    private final ArrayDeque<String> pendingCues = new ArrayDeque<>();
    private TextToSpeech textToSpeech;
    private boolean speechReady;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        textToSpeech = new TextToSpeech(this, status -> {
            if (status != TextToSpeech.SUCCESS) {
                Log.e(TAG, "Android text-to-speech initialization failed");
                return;
            }
            int language = textToSpeech.setLanguage(Locale.US);
            if (language == TextToSpeech.LANG_MISSING_DATA
                    || language == TextToSpeech.LANG_NOT_SUPPORTED) {
                Log.e(TAG, "English text-to-speech is unavailable on this headset");
                return;
            }
            runOnUiThread(() -> {
                speechReady = true;
                while (!pendingCues.isEmpty()) {
                    speak(pendingCues.removeFirst());
                }
            });
        });
    }

    public void announceRecordingEvent(String event) {
        final String phrase;
        if ("started".equals(event)) {
            phrase = "recording starts";
        } else if ("ended".equals(event)) {
            phrase = "recording ends";
        } else {
            return;
        }
        runOnUiThread(() -> {
            if (speechReady) {
                speak(phrase);
            } else if (pendingCues.size() < 4) {
                pendingCues.addLast(phrase);
            }
        });
    }

    private void speak(String phrase) {
        if (textToSpeech != null) {
            textToSpeech.speak(phrase, TextToSpeech.QUEUE_ADD, null,
                    "recording-" + System.nanoTime());
        }
    }

    @Override
    protected void onDestroy() {
        if (textToSpeech != null) {
            textToSpeech.stop();
            textToSpeech.shutdown();
            textToSpeech = null;
        }
        super.onDestroy();
    }
}
