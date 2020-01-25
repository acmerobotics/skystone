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

                    drive.goToPosition(28);
                    state++;

                    break;

                case 1:

                    if(drive.atLinearPos()){
                        foundationMover.moveToGrab();

                        state++;

                    }

                    break;

                case 2:

                    drive.goToPosition(34);

                    state++;

                    break;

                case 3:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }


                case 4:

                    if (!timeReset) {
                        time.reset();
                        timeReset = true;
                    }

                    if(time.seconds() < 1) {
                        drive.strafeRight();

                    } else {

                        drive.stopMotors();
                        drive.resetAngle();
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


                    /*

                case 6:

                    foundationMover.moveToStore();

                    //might have to do thread.sleep here cause it might not release in time before it has to move back

                    state++;

                    break;


                case 7:

                    drive.resetLinearPos();

                    //drive to  the bridge and such




                    //TODO add the init sequence with the lift and such.
*/

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
