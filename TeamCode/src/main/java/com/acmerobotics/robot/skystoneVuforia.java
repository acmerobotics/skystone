package com.acmerobotics.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;

public class skystoneVuforia {

    private String key = "";

    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;

    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private boolean targetVisibale = false;

    public double x = 0;
    public double y = 0;
    public double z = 0;
    public double rX = 0;
    public double rY = 0;
    public double rZ = 0;

    public OpenGLMatrix pose;

    private boolean skystoneReached = false;
    private boolean moveToBrige = false;

    private int state = 0;


    public skystoneVuforia(){
    }

    public void init(HardwareMap hardwareMap){
        int camaeraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName()); // will allow the camera output to be displayed on the phone screen

        // setup parameters
        parameters = new VuforiaLocalizer.Parameters(camaeraMonitorViewId);
        parameters.vuforiaLicenseKey = key; // need to enter a vuforia licence key
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // behind screen

        vuforia = ClassFactory.getInstance().createVuforia(parameters); // creates vuforia localizer with set parameters

        // load targets from assets
        visionTargets = vuforia.loadTrackablesFromAsset("Skystone"); // vuMarks from the rController assets

        // instantiate targets
        // check FtcRobotController/assets/xml file
        VuforiaTrackable sksystone = visionTargets.get(0); // # changes based on desired vuMark from targets
        target = sksystone;
        sksystone.setName("skystone"); // Skystone vuMark

    }


    public void activate(){
        visionTargets.activate();
    }


    public void deactivate(){
        visionTargets.deactivate();
    }


    // if one target is visible
    public void searchForTarget(VuforiaTrackable target){
            // the listener is the software that is looking at the data coming from the camera to check
            // for vumarks
        if (((VuforiaTrackableDefaultListener)target.getListener()).isVisible()) { // is vumark visible
            targetVisibale = true;
            pose = ((VuforiaTrackableDefaultListener)target.getListener()).getPose(); // get vumark pose (x, y, heading)
        }
    }


    public void getLocationFromTarget(){
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            // ^ returns the "translation component of the transformation (pose in this case)"
            // inside the pose matrix there is are translation coordinates (xyz) that are extracted
            // and formatted with normalized3D() so it returns the xyz coordinates that are useful to us
            Orientation rot = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // get XYZ
            x = trans.get(0);
            y = trans.get(1);
            z = trans.get(2);

            // Heading, Pitch, Roll
            rX = rot.firstAngle;
            rY = rot.secondAngle;
            rZ = rot.thirdAngle;
        }
    }


    public void goToSkystone(Drive drive, Intake intake){
        // goal is to get all xyz to 0

        // get closer to skystone and adjust heading to stay parallel while moving

        intake.leftOpen();
        intake.setIntakePower(1);

        searchForTarget(target);

        if (targetVisibale && !skystoneReached){
            getLocationFromTarget();

            // go to target
            correction(drive);
        }

        else if (skystoneReached){
            moveToBrige = true;
        }

    }


    public double conroller(double error, double p){
        double correction = error * p;

        if (correction > 1){
            correction = 1;
        }

        if (correction < -1){
            correction = -1;
        }

        return correction;
    }

    public void correction(Drive drive){
        double xError02 = 0 + x;
        double xError13 = 0 + x;

        double yError = 0 + y;

        double rError = 0 + rX;

        //  I don't know which side of the X or Y axis is positive or negative so I will assume that
        // positive X axis is to the robots right and the positive Y is in front of the robot

        if (xError02 > 0){
            // when stone is on right prevent movement to left
            xError13 = 0;
        }

        if (xError02 < 0){
            // when stone is on left prevent movement to right
            xError02 = 0;
        }

        double motorPower02 = conroller(xError02, 5) + conroller(yError, 5);
        double motorPower13 = conroller(xError13, 5) + conroller(yError, 5);

        // if heading error is very large correct it before moving on
        if (Math.abs(rError) >  20){
            //todo heading correction //////////////////////////////////////
        }

        else {
            drive.motors[0].setPower(motorPower02);
            drive.motors[1].setPower(motorPower13);
            drive.motors[2].setPower(motorPower02);
            drive.motors[3].setPower(motorPower13);
        }

        if (yError < 1 && xError02 < 1){
            skystoneReached = true;
        }
    }


    public void moveToBridge(Drive drive){
        // back up and move to bridge

        if (moveToBrige) {

            switch (state) {
                case 0:
                    drive.resetEncoders();
                    state++;

                case 1:
                    // back up
                    drive.goToPosition(-5, 0.65);
                    state++;

                case 2:
                    if (drive.atLinearPos()) {
                        drive.stopMotors();
                        state++;
                    }
                    break;

                case 3:
                    // todo strafe to left or right
                    deactivate();
                    state++;
            }
        }
    }


    public void telemetry(Telemetry telemetry){
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addLine();
        telemetry.addData("r", rX);

        telemetry.update();
    }


}
