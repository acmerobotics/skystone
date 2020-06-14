package com.acmerobotics.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/***
 * This probably has no particle use but it would make driving very fun (for me at least). This
 * should just activate the phone camera and display it on the DC phone. It will turn a 3rd person
 * view of the robot into a first person view.
 */

/***
 * there is probably a better way of doing this then by using vuforia
 */


public class FirstPersonRobotDrive {

    // choose which camera to use
    public static VuforiaLocalizer.CameraDirection cameraSide = BACK;

    public FirstPersonRobotDrive(OpMode opMode){
        // show camera on DC phone
        int CameraMonitorView = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorView", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(CameraMonitorView);

        // set which camera to use
        parameters.cameraDirection = cameraSide;

    }



}
