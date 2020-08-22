package com.acmerobotics.openCVvision;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.opencv.android.JavaCamera2View;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

public class imgCapture {

    /*
    I'm hoping this will capture an image on the phone
     */

    public Mat img;
    public int cameraNum = 0; // find right camera number
    public boolean imgFound;

    private VideoCapture capture = new VideoCapture(cameraNum); // find right camera number


    public void getImg(){

        Mat img = new Mat();

        imgFound = capture.read(img);

        capture.release();

        this.img = img;

    }

    public void saveImg(){
        Imgcodecs.imwrite("openCV_img_capture_test.png", img); // I only save the img so it can help test and trouble shoot
    }
}
