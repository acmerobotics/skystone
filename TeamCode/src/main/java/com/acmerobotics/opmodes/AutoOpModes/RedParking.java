package com.acmerobotics.opmodes.AutoOpModes;


import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.acmerobotics.robot.Intake;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Parking")
public class RedParking extends LinearOpMode {


    private boolean moveToFoundation = false;
    private boolean strafeRight = false;
    private int state;
    private boolean timeReset;

    // FtcDashboard dashboard  = FtcDashboard.getInstance();
    //Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);
        liftEncoder lift = new liftEncoder(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        armEncoder arm = new armEncoder(hardwareMap);
        ElapsedTime time = new ElapsedTime();

        state = 0;
  
        drive.resetEncoders();
        drive.resetAngle();
        drive.update();

        lift.init();
        arm.init();

        lift.resetEncoder();
        arm.resetEncoder();

        arm.runTo(100); // gets arm out of the intake's way

        intake.rightFullyOpen();

        waitForStart();

        intake.leftFullyOpen();

        while(!isStopRequested()) {

            switch (state) {

                case 0:

                    drive.goToPosition(25, 0.25);

                    if (drive.atLinearPos()){
                        drive.stopMotors();
                    }

                    if(lift.bottomSet){
                        state++;
                    }

                    else if (!lift.bottomSet){

                        lift.tightenLiftString();

                        lift.goToBottom();
                    }

                    break;


//                case 1:
//
//                    drive.goToPosition(10, 0.25);
//                    state++;
//                    break;
//
//                case 2:
//
//                    if (drive.atLinearPos()) {
//                        drive.stopMotors();
//
//                        state++;
//                    }
//                    break;
            }

            telemetry.addData("state", state);
            telemetry.addData("lift ready", lift.bottomSet);
            telemetry.update();

        }
    }
}