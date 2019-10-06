package com.acmerobotics.util;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    public static double MAX_I = 0.25;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    private double lastTimeStamp;
    private double lastError;
    private double setPoint;
    private double upperBound;
    private double lowerBound;
    private boolean updated;
    private boolean isBounded;

    public PIDController(double kP, double kI, double kD){
        PIDController.kP = kP;
        PIDController.kI = kI;
        PIDController.kD = kD;
    }

    public void setPoint(double setPoint){
        this.setPoint = setPoint;
    }

    public void setBounds(double upperBound, double lowerBound){
        this.upperBound = upperBound;
        this.lowerBound = lowerBound;
        isBounded = true;
    }

    public double update(double position) {
        position = Angle.norm(position);
        double error = position - setPoint;
        double p = error * kP;

        if (Math.abs(position - setPoint) > Math.PI) {
            position += Math.copySign(Math.PI * 2, setPoint);

        }

            if (!updated) {
                lastTimeStamp = System.currentTimeMillis();
                lastError = error;
                updated = true;
                return p;
            }

            double now = System.currentTimeMillis();
            double dt = now - lastTimeStamp;
            lastTimeStamp = now;

            double intergrand = dt * error;
            double i = intergrand * kI;
            i = Range.clip(i, -MAX_I, MAX_I);

            double derivative = (error - lastError / dt);
            lastError = error;
            double d = derivative * kD;

            return p + i + d;

    }
}
