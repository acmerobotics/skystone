package com.acmerobotics.openCVvision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="OpenCV")
public class captureOpMode extends LinearOpMode {

    /*
    I will use this to test if I can actually capture an image
     */

    @Override
    public void runOpMode(){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        imgCapture camera = new imgCapture();

        waitForStart();

        telemetry.addLine("start pressed");

        while (!isStopRequested()){

            camera.getImg();

            camera.saveImg();

            telemetry.addLine("was img found " + camera.imgFound);

            telemetry.addLine("is Img empty " + camera.img.empty());

            telemetry.addLine("program end");

            telemetry.update();
        }
    }
}
