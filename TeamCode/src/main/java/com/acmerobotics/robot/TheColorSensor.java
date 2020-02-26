package com.acmerobotics.robot;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class TheColorSensor {

    private  ColorSensor colorSensor;

    private int scaleFactor = 255;

    public float[] hsvValues = new float[3];

    public static double skystoneHue = 120; // was 100, I will probably need to edit this

    public TheColorSensor(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public void HSV(){
        Color.RGBToHSV((colorSensor.red() * scaleFactor), (colorSensor.green() * scaleFactor),(colorSensor.blue() * scaleFactor), hsvValues);
    }


    public boolean isSkystoneHue(){
        if(hsvValues[0] > skystoneHue){
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
