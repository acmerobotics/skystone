package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.RobomaticTesting.generalizedDrive;
import com.acmerobotics.RobomaticTesting.roboRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleUsingGenDrive")
public class TeleOpUsingGenDrive extends LinearOpMode {

    public int state = 0;

    @Override
    public void runOpMode(){
        roboRobot robot = new roboRobot(this);
        generalizedDrive drive = new generalizedDrive(robot, this);

        drive.stopAndResetMotors();
        drive.resetAngle();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while(!isStopRequested()){
            switch (state){
                case 1:
                    drive.strafeLeft(10);
                    state++;

                case 2:
                    if(drive.atStrafePosition()){
                        drive.stopAndResetMotors();
                        state++;
                    }
                    break;

                case 3:
                    drive.moveForward(29);
                    state++;

                case 4:
                    if(drive.atYPosition()){
                        drive.stopAndResetMotors();
                        // grab foundation
                        drive.resetAngle();
                        state++;
                    }
                    break;

                case 5:
                    drive.turnRight(179);
                    state++;

                case 6:
                    if (drive.atTurningPosition()){
                        drive.stopAndResetMotors();
                        state++;
                    }
                    break;

                case 7:
                    drive.moveForward(20);
                    state++;

                case 8:
                    if (drive.atYPosition()){
                        drive.stopAndResetMotors();
                        state++;
                    }
                    break;

                case 9:
                    // move foundation
                    state++;

                case 10:
                    drive.moveBack(2);
                    state++;

                case 11:
                    if (drive.atYPosition()){
                        drive.stopAndResetMotors();
                        drive.resetAngle();
                        state++;
                    }
                    break;

                case 12:
                    drive.turnLeft(80);
                    state++;

                case 13:
                    if (drive.atTurningPosition()){
                        drive.stopAndResetMotors();
                        state++;
                    }
                    break;

                case 14:
                    drive.moveForward(42);
                    state++;

                case 15:
                    if (drive.atYPosition()){
                        drive.stopAndResetMotors();
                        state++;
                    }
                    break;

                case 16:
                    // init proccess
            }
        }

    }
}
