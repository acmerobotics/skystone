package com.acmerobotics.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class JoystickTransform {

    public static double exponent = 2;

    public static double r;
    public static double omega;
    public static double sigOmega;

    private boolean ramping = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();


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

        r = original.vec().norm();
        omega = original.getHeading();
        sigOmega = Math.signum(omega);
        dashboard.sendTelemetryPacket(packet);


        switch (mode) {

            case EXPONENTIAL:
                r = Math.pow(r, exponent);
                omega = Math.pow(omega, exponent);
                packet.put("r", r);
                packet.put("omega", omega);
                dashboard.sendTelemetryPacket(packet);
                break;

            case LINEAR:
                break;
        }

        Pose2d command = new Pose2d(original.vec().times(r / original.vec().norm()), omega * sigOmega);
        if (r == 0){
            command = new Pose2d(0, 0, omega * sigOmega);

        }
        return command;


    }



}
