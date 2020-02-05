package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Foundation")
public class BlueFoundation extends LinearOpMode {

    private int state;


    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);

        state = 0;

        drive.resetEncoders();
        drive.resetAngle();
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

                    drive.goToStrafingPos(24, 0.5, "left");
                    state++;

                    break;

                case 1:

                    if(drive.atStrafingPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;

                case 2:

                    drive.goToPosition(30, 0.25);
                    state++;

                    break;

                case 3:

                    if(drive.atLinearPos()){
                        foundationMover.moveToGrab();

                        Thread.sleep(1500);

                        state++;

                    }

                    break;


                case 4:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }


                case 5:

                    drive.setDegrees(-179);

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

                    drive.resetEncoders();
                    drive.resetLinearPos();

                    drive.goToPosition(15, 0.5);

                    state++;

                    break;


                case 7:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;

                case 8:

                    foundationMover.moveToStore();

                    state++;

                    break;


                case 9:

                    drive.resetEncoders();
                    drive.resetLinearPos();


                    drive.goToPosition(-3, -0.5);

                    state++;

                case 10:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;

                case 11:

                    drive.resetAngle();

                    state++;

                    break;

                case 12:

                    drive.setDegrees(80);

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

                    drive.goToPosition(44, -0.5);

                    state++;

                    break;

                case 14:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;

                case 15:

                    //TODO add the init sequence with the lift and such.

            }





            telemetry.addData("state", state);
            telemetry.addData("current pos", drive.getCurrentPos());
            telemetry.addData("target pos", drive.getTargetMotorPos());
            telemetry.addData("motors stopped", drive.areMotorsStopped());
            telemetry.addData("current angle", drive.getCurrentAngle());
            telemetry.update();

        }




    }

}

