package com.acmerobotics.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class JoystickTransform {

    //TODO finish this before thursday

    public static double exponent = 2;

    private boolean ramping = false;

    public MODE mode = MODE.LINEAR;

    public enum MODE {
        LINEAR,
        EXPONENTIAL
    }

    public JoystickTransform() {

    }

    public void setMode(MODE mode) {
        this.mode = mode;
    }

    public MODE getMode() {
        return mode;
    }

    public void setRamping(boolean enable){
        ramping = enable;

    }

    public boolean getRamping(){
        return ramping;
    }

    public Pose2d transform(Pose2d original) {

        double r = original.vec().norm();
        double omega = original.getHeading();
        double sigomega = Math.signum(omega);

        switch (mode) {

            case EXPONENTIAL:
                r = Math.pow(r, exponent);
                omega = Math.pow(omega, exponent);

                break;

            case LINEAR:
                break;
        }

        Pose2d command = new Pose2d(original.vec().times(r / original.vec().norm()), omega * sigomega);
        if (r == 0){
            command = new Pose2d(0, 0, omega * sigomega);

        }
        return command;


    }



}
