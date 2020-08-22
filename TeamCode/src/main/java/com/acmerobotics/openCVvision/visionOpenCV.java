package com.acmerobotics.openCVvision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class visionOpenCV {

    /*
    classing showing the basic processes of openCV in java
     */

    public double X;
    public double Y;
    public double W;
    public double H;

    public double lowerH = 50;
    public double upperH = 95;

    private Scalar lower = new Scalar(lowerH, 50, 50);
    private Scalar upper = new Scalar(upperH, 255, 255);

    // read img (comes in BGR)
    private Mat rawImg = Imgcodecs.imread("com/acmerobotics/openCVvision/darkCup.png");

    private Size kernalSize = new Size(10, 10);
    public Mat kernal = Mat.ones(kernalSize, rawImg.type());

    public List<MatOfPoint> contours = new ArrayList<>();


    public void proccessImg(Mat img){

        // create binary img by threshing then morphing
        Mat binaryImg = isolate(img);

        // detect ROI with contours
        Rect ROI = detect(binaryImg);

        // extract rect info created by contours
        X = ROI.x; // top left
        Y = ROI.y; // top left
        W = ROI.width;
        H = ROI.height;
    }


    private Mat isolate(Mat img){
        // to hsv
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);

        // thresh
        Core.inRange(img, lower, upper, img);

        // open = erode then dilate
        Imgproc.morphologyEx(img, img, Imgproc.MORPH_OPEN, kernal);

        return img;
    }


    private Rect detect(Mat img){
        Mat hierarchy = new Mat();

        // find contours
        Imgproc.findContours(img, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // I test to make sure there was only one contour but this won't always
        // be the case
        Mat contour = contours.get(0);

        Rect rect = Imgproc.boundingRect(contour);

        return rect; // with this I can get x, y, w, h of rect
    }

}
