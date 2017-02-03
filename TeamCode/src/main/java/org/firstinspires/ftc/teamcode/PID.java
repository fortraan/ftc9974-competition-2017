package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

/**
 * PID controller programmed in Java
 *
 * @author christopher.johnson
 * Created by christopher.johnson on 10/5/16.
 * TODO generate more Javadocs
 *
 * Credit:
 * Brett Beauregard, creator of PID_v1, an Arduino library that this is based on
 */
public class PID {
    public enum MODES {
        AUTO_TUNE,
        MANUAL,
        MANUAL_TUNE
    }

    public double input;
    public double output;

    private MODES _mode;
    private double _min;
    private double _max;
    private double _setpoint;
    private double P;
    private double I;
    private double D;
    private double ITerm;
    private double lastInput;
    private long now;
    private long lastTime;
    private long deltaTime;
    private long sampleTime = 100;

    /**
     * Make a new PID object.
     *
     * The tunings must be manual, but
     * http://innovativecontrols.com/blog/basics-tuning-pid-loops is a good reference
     * @param min minimum output
     * @param max maximum output
     * @param setpoint target value the PID tries to achieve
     * @param Kp P coefficient
     * @param Ki I coefficient
     * @param Kd D coefficient
     * @param mode Loop mode, yet to be implemented
     * @param in starting input
     */
    PID(double min, double max, double setpoint, double Kp, double Ki, double Kd, MODES mode, double in) {
        _min = min;
        _max = max;
        setSetpoint(setpoint);
        _mode = mode;
        setTunings(Kp, Ki, Kd);
        lastTime = SystemClock.currentThreadTimeMillis();
        input = in;
    }

    PID(double min, double max, double setpoint, double Kp, double Ki, double Kd, double input) {
        this(min, max, setpoint, Kp, Ki, Kd, MODES.MANUAL, input);
    }

    PID(double setpoint, double Kp, double Ki, double Kd, double input) {
        this(setpoint - (input*2), setpoint + (input*2), setpoint, Kp, Ki, Kd, MODES.MANUAL, input);
    }

    public void setTunings(double p, double i, double d) {
        double SampleTimeInSecs = ((double) sampleTime) / 1000;
        P = p;
        I = i * SampleTimeInSecs;
        D = d / SampleTimeInSecs;
    }

    public void setSetpoint(double setpoint) {
        _setpoint = setpoint;
    }

    public boolean DoCycle() {
        now = SystemClock.currentThreadTimeMillis();
        deltaTime = now - lastTime;
        if (deltaTime >= sampleTime) {
            double error = _setpoint - input;
            ITerm += I * error;
            ITerm = (ITerm > _max) ? _max : ITerm;
            ITerm = (ITerm < _min) ? _min : ITerm;
            double dInput = input - lastInput;

            // Where the magic happens
            double _output = P * error + ITerm - D * dInput;

            _output = (_output > _max) ? _max : _output;
            _output = (_output < _min) ? _min : _output;
            output = _output;
            return true;
        }
        return false;
    }

    public void setInput(double input) {
        this.input = input;
    }

    public double getSetpoint() {
        return _setpoint;
    }

    public String outputAsString() {
        return String.valueOf(output);
    }
}
