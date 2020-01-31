package com.acmerobotics.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TheColorSensor {

    ColorSensor colorSensor;

    public int scaleFactor = 255;

    public float[] hsvValues = new float[3];

    public TheColorSensor(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public float[] HSV(){
        Color.RGBToHSV((colorSensor.red() * scaleFactor), (colorSensor.green() * scaleFactor),(colorSensor.blue() * scaleFactor), hsvValues);

        return hsvValues;
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

}
