package com.acmerobotics.opmodes;

import com.acmerobotics.robot.KotlinArm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class javaForKotlinOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        KotlinArm arm = new KotlinArm(hardwareMap); // a kotlin class that can be used as an ordinary
                                                    // Java object

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (!isStopRequested()){
            arm.runTo(100);
        }
    }
}
