package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Foundation")
public class RedFoundation extends LinearOpMode {

    private boolean moveToFoundation = false;
    private boolean strafeRight = false;
    private boolean grabbedFoundation = false;
    private int state;
    private boolean timeReset;

    // FtcDashboard dashboard  = FtcDashboard.getInstance();
    //Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);
        ElapsedTime time = new ElapsedTime();

        state = 0;
        timeReset = false;

        drive.resetEncoders();
        drive.resetAngle();
        time.reset();
        //drive.resetLinearPos();
        drive.update();

        telemetry.addData("state", state);
        telemetry.addData("current pos", drive.getCurrentPos());
        telemetry.addData("target pos bool", drive.returnAtTargetPos());
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        while(!isStopRequested()) {

            switch (state) {

                case 0:

                    drive.goToPosition(26);
                    state++;

                    break;

                case 1:

                    if(drive.atLinearPos()){
                        drive.stopMotors();
                        state++;

                    }

                    break;

                case 2:

                    foundationMover.moveToGrab();

                    Thread.sleep(600);

                    state++;



                    break;


                case 3:

                    if (!timeReset) {
                        time.reset();
                        timeReset = true;
                    }

                    if(time.seconds() < 2) {
                        drive.strafeRight();

                    } else {

                        state++;
                    }


                    break;

                case 4:

                    drive.setDegrees(-265);

                    drive.getRadians();

                    if(drive.getAngle() == 0) {
                        drive.clockwise();
                    }

                    if(drive.getRadians() > 0) {

                        if(drive.getAngle() < drive.getRadians()){
                            drive.counterClockwise();

                        } else {

                            drive.stopMotors();
                            state++;
                        }

                    } else {

                        if(drive.getAngle() > drive.getRadians()){
                            drive.clockwise();

                        } else {

                            drive.stopMotors();
                            state++;
                        }

                    }

                    break;


            }

            telemetry.addData("state", state);
            telemetry.addData("current pos", drive.getCurrentPos());
            telemetry.addData("target pos", drive.getTargetPos());
            telemetry.addData("linear pos", drive.atLinearPos());
            telemetry.update();

        }


    }
}
