package org.firstinspires.ftc.teamcode.RnD;

import android.app.Activity;
import android.content.Context;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.R;

import java.util.Collections;
import java.util.Map;

/**
 * Created by christopher.johnson on 2/25/17.
 */

public class PusheenDisplay {
    private enum STATE {
        IDLE,
        SPEAKING,
        THINKING
    }

    private FtcRobotControllerActivity activity;
    private Context context;

    public PusheenDisplay (Context appContext) {
        context = appContext;
        activity = (FtcRobotControllerActivity) context;


    }
}
