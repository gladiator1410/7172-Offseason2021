package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    public double lastVal = 0;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double iTerm = 0;
    public double fTerm = 0;
    public double out = 0;
    public ElapsedTime pidTimer = new ElapsedTime();
    public double tDelta = 0.01;

    public void setPIDF(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        fTerm = f;
        iTerm = 0;
    }

    public void zeroI() { iTerm = 0; }

    public double calc(double val, double setPoint) {
        if (pidTimer.seconds() < tDelta) return out;
        pidTimer.reset();
        double err = val - setPoint;
        double dVal = lastVal - val;
        double pTerm = kP * err;
        iTerm = iTerm + kI * err;
        double dTerm = -kD * dVal;
        lastVal = val;
        out = pTerm + iTerm + dTerm + fTerm;
        return out;
    }

}

