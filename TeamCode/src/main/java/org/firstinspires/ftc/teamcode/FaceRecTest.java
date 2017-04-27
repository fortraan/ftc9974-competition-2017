package org.firstinspires.ftc.teamcode;


import android.annotation.TargetApi;
import android.content.Context;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.InputConfiguration;
import android.os.Build;
import android.os.Handler;
import android.os.Message;
import android.support.annotation.NonNull;
import android.view.Surface;
import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Collections;

/**
 * Created by christopher.johnson on 2/25/17.
 */

public class FaceRecTest extends LinearOpMode {
    private CameraManager manager;
    private CameraDevice cam;
    private boolean cameraReady = false;
    private CameraDevice.StateCallback stateCallback;
    @TargetApi(Build.VERSION_CODES.LOLLIPOP)
    @Override
    public void runOpMode() throws InterruptedException {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            manager = (CameraManager) hardwareMap.appContext.getSystemService(Context.CAMERA_SERVICE);
        }

        stateCallback = new CameraDevice.StateCallback() {
            @Override
            public void onOpened(@NonNull CameraDevice camera) {
                cam = camera;
                cameraReady = true;
            }

            @Override
            public void onDisconnected(@NonNull CameraDevice camera) {

            }

            @Override
            public void onError(@NonNull CameraDevice camera, int error) {

            }
        };

        CameraManager.AvailabilityCallback availabilityCallback = new CameraManager.AvailabilityCallback() {
            @Override
            public void onCameraAvailable(@NonNull String cameraId) {
                super.onCameraAvailable(cameraId);

                try {
                    manager.openCamera(cameraId, stateCallback, null);
                } catch (CameraAccessException e) {
                    e.printStackTrace();
                }
            }
        };

        manager.registerAvailabilityCallback(availabilityCallback, null);

        while (!isStopRequested() && !isStarted()) {
            if (cameraReady) {
                telemetry.addLine("Camera ready");
                telemetry.update();
            }
            idle();
        }

        CameraCaptureSession.StateCallback sessionCallback = new CameraCaptureSession.StateCallback() {
            @Override
            public void onConfigured(CameraCaptureSession session) {

            }

            @Override
            public void onConfigureFailed(CameraCaptureSession session) {

            }
        };

        /*try {
            cam.createCaptureSession(Collections.<Surface>emptyList(), );
        } catch (CameraAccessException e) {
            e.printStackTrace();
            requestOpModeStop();
        }*/
    }
}
