package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;


import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class AngleCorrector {

    public static double Pcoefficient = 0.025;
    public static double defaultMotorPower = -0.28;

    public double error;
    public double newPower;

    public Drive drive;

    public AngleCorrector(HardwareMap hardwareMap){
        drive = new Drive(hardwareMap, false);
    }

    public void setZero(){
        drive.resetAngle();
        drive.setDegrees(0);
    }


    public double getError(){
        return Math.round(drive.getAngle() - drive.getDegrees());
    }


    public void setError(){
        error = getError();
    }


    public double Pcontroller(){
        double output = Pcoefficient * error;
        return output;
    }


    public void setNewPower(){
        setError();

        double correctionPower = Pcontroller();

        newPower = defaultMotorPower - (Math.copySign(correctionPower, defaultMotorPower));

        drive.motors[2].setPower(newPower);
        drive.motors[3].setPower(newPower);
    }
}
