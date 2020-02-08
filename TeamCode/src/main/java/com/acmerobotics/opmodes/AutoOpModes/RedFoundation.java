package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Foundation")
public class RedFoundation extends LinearOpMode {
    private int state;

    // FtcDashboard dashboard  = FtcDashboard.getInstance();
    //Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);
        ElapsedTime time = new ElapsedTime();


        state = 0;
        drive.resetEncoders();
        drive.resetAngle();
        drive.resetEncoderOmni();
        time.reset();
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

                    drive.goToPosition(29, 0.25);
                    state++;

                    break;

                case 1:

                    if(drive.atLinearPos()){
                        foundationMover.moveToGrab();

                        Thread.sleep(1500);

                        state++;

                    }

                    break;


                case 2:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;



                case 3:

                    drive.resetEncoderOmni();
                    drive.resetStrafingPos();

                    state++;

                    break;


                case 4:

                    drive.goToStrafingPos(-40, 0.5, "right");
                    state++;

                    break;

                case 5:

                    if(drive.atStrafingPos()){
                        drive.stopMotors();
                        drive.resetAngle();

                        state++;
                    }

                    break;


                case 6:



                    drive.setDegrees(179);


                    if(drive.getAngle() == 0) {
                        drive.clockwise();
                    }

                    if(drive.getDegrees() > 0) {

                        if(drive.getAngle() < drive.getDegrees()){
                            drive.counterClockwise();

                        } else {

                            drive.stopMotors();
                            state++;
                        }

                    } else {

                        if(drive.getAngle() > drive.getDegrees()){
                            drive.clockwise();

                        } else {

                            drive.stopMotors();
                            state++;
                        }

                    }

                    break;

                case 7:

                    drive.resetEncoders();
                    drive.resetLinearPos();

                    drive.goToPosition(15, 0.75);

                    state++;

                    break;

                case 8:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;

                case 9:

                    foundationMover.moveToStore();

                    state++;

                    break;


                case 10:

                    time.reset();

                    state++;

                    break;

                case 11:

                    if(time.seconds() < 0.2){
                        drive.moveBack();


                    } else {

                        drive.resetAngle();
                        state++;
                    }

                    break;


                case 12:

                    drive.setDegrees(-75);

                    drive.getDegrees();

                    if(drive.getAngle() == 0) {
                        drive.clockwise();
                    }

                    if(drive.getDegrees() > 0) {

                        if(drive.getAngle() < drive.getDegrees()){
                            drive.counterClockwise();

                        } else {

                            drive.stopMotors();
                            state++;
                        }

                    } else {

                        if(drive.getAngle() > drive.getDegrees()){
                            drive.clockwise();

                        } else {

                            drive.stopMotors();
                            state++;
                        }

                    }

                    break;


                case 13:

                    drive.resetEncoders();
                    drive.resetLinearPos();

                    drive.goToPosition(43, -0.5);

                    state++;

                    break;

                case 14:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;


                case 15:

                    // add in all of the pre init stuff

            }



            telemetry.addData("state", state);
            telemetry.addData("current pos", drive.getCurrentPos());
            telemetry.addData("target pos", drive.getTargetMotorPos());
            telemetry.addData("motors stopped", drive.areMotorsStopped());
            telemetry.addData("current angle", drive.getCurrentAngle());
            telemetry.addData("current omni pos inches", drive.getCurrentTrackerPosInches());
            telemetry.addData("target omni pos", drive.getTargetOmniPos());
            telemetry.update();

        }


    }
}
