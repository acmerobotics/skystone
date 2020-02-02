package com.acmerobotics.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TheColorSensor {

    private  ColorSensor colorSensor;

    private int scaleFactor = 255;

    public float[] hsvValues = new float[3];

    public TheColorSensor(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public void HSV(){
        Color.RGBToHSV((colorSensor.red() * scaleFactor), (colorSensor.green() * scaleFactor),(colorSensor.blue() * scaleFactor), hsvValues);
    }


    public boolean isSkystoneHue(){
        if(hsvValues[0] > 30){
            return true;
        }

        else{
            return false;
        }
    }


    public boolean isSkystoneSat(){
        if(hsvValues[1] < 0.35){
            return true;
        }

        else{
            return false;
        }
    }


    public double RED(){
        return colorSensor.red();
    }

    public double GREEN(){
        return colorSensor.green();
    }

    public double BLUE(){
        return colorSensor.blue();
    }

    public int hue(){
        return colorSensor.argb();
    }

}
