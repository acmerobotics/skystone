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
        Drive drive = new Drive(hardwareMap, false);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);
        ElapsedTime time = new ElapsedTime();

        state = 0;
        timeReset = false;

        drive.resetEncoders();
        drive.resetAngle();
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

                    drive.goToPosition(28, 0.25);
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


                case 3:

                    if (!timeReset) {
                        time.reset();
                        timeReset = true;
                    }

                    if(time.seconds() < 0.7) {
                        drive.strafeRight();

                    } else {

                        drive.stopMotors();
                        drive.resetAngle();
                        timeReset = false;
                        state++;
                    }


                    break;

                case 4:


                    if(!timeReset){
                        time.reset();
                        timeReset = true;
                    }

                    if(time.seconds() < 1) {
                        drive.moveBack();

                    } else {

                        drive.stopMotors();
                        drive.resetAngle();
                        timeReset = false;
                        state++;
                    }

                    break;





                case 5:

                    drive.setDegrees(179);

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

                case 6:

                    if(!timeReset){
                        time.reset();
                        timeReset = true;
                    }

                    if(time.seconds() < 2) {
                        drive.moveForward();

                    } else {

                        drive.stopMotors();
                        drive.resetAngle();
                        timeReset = false;
                        state++;
                    }

                    break;



                case 7:

                    foundationMover.moveToStore();

                    state++;

                    break;


                case 8:

                    drive.resetEncoders();
                    drive.resetLinearPos();


                    drive.goToPosition(-3, -0.5);

                    state++;

                case 9:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;

                case 10:

                    // add in the turing and stuff to get under the bridge







                    //TODO add the init sequence with the lift and such.


            }



            telemetry.addData("state", state);
            telemetry.addData("current pos", drive.getCurrentPos());
            telemetry.addData("target pos", drive.getTargetPos());
            telemetry.addData("motors stopped", drive.areMotorsStopped());
            telemetry.addData("current angle", drive.getCurrentAngle());
            telemetry.update();

        }


    }
}
