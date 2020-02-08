package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="omni test")
public class EncoderOmniTest extends LinearOpMode {
    private int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);

        drive.resetEncoderOmni();

        telemetry.addData("current encoder pos", drive.getCurrentTrackerPosInches());
        telemetry.update();

        state = 0;

        waitForStart();

        telemetry.clearAll();


        while(!isStopRequested()){

            switch (state) {

                case 0:
                    drive.goToStrafingPos(10, 0, "right");

                    state++;

                    break;

                case 1:

                    drive.getCurrentTrackerPosInches();
                    drive.getCurrentTrackerPosTicks();

                    break;

            }



            telemetry.addData("current encoder pos inches", drive.getCurrentTrackerPosInches());
            telemetry.addData("current encoder pos ticks", drive.getCurrentTrackerPosTicks());
            telemetry.addData("target encoder position", drive.getTargetOmniPos());
            telemetry.update();


        }

    }
}
