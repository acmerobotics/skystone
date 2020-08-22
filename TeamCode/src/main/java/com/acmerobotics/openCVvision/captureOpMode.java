package com.acmerobotics.openCVvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class captureOpMode extends LinearOpMode {

    /*
    I will use this to test if I can actually capture an image
     */

    @Override
    public void runOpMode(){

        imgCapture camera = new imgCapture();

        waitForStart();


        telemetry.addLine("start pressed");

        camera.getImg();

        camera.saveImg();

        telemetry.addLine("was img found " + camera.imgFound);

        telemetry.addLine("is Img empty " + camera.img.empty());

        telemetry.addLine("program end");

        telemetry.update();

//        while (!isStopRequested()){
//
//        }
    }
}
