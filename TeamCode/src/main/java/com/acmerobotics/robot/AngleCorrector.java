package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;


import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class AngleCorrector {

    public static double Pcoefficient = 0.025; // 0.025 is for moving forward and back; for strafing use 0.02

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


    private double getError(){
        return drive.getAngle() - drive.getDegrees();
    }


    private void setError(){
        error = getError();
    }


    private double Pcontroller(){
        double output = Pcoefficient * error;
        return output;
    }


    public void setNewPower(double defaultPower, int motorNum){
        setError();

        double correctionPower = Pcontroller();

        double changeSign = Math.copySign(1, defaultPower); // 1 or -1

        if (defaultPower != 0) {

            newPower = defaultPower - (correctionPower * changeSign);
        }

        else {
            if (motorNum == 0 || motorNum == 1) {
                newPower = defaultPower - correctionPower;
            }

            else { // motors 2 and 3
                newPower = defaultPower + correctionPower;
            }
        }

        drive.motors[motorNum].setPower(newPower);
    }

}
