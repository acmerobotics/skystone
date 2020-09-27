package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

@TeleOp(name="Java for Kotlin")
public class javaForKotlinOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (!isStopRequested()){

            int id = hardwareMap.appContext
                    .getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


            VideoCapture capture = new VideoCapture(id);

            Mat img = new Mat();

            capture.read(img);

            //telemetry.addData("", );

            telemetry.update();
        }
    }
}
