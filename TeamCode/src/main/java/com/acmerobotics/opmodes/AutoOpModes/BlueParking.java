package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.liftEncoder;
import com.acmerobotics.robot.Intake;
import com.acmerobotics.robot.armEncoder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Parking")
public class BlueParking extends LinearOpMode {


    private boolean moveToFoundation = false;
    private boolean strafeRight = false;
    private int state;
    private boolean timeReset;


    // FtcDashboard dashboard  = FtcDashboard.getInstance();
    //Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        liftEncoder lift = new liftEncoder(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        armEncoder arm = new armEncoder(hardwareMap);
        Drive drive = new Drive(hardwareMap, false);
        ElapsedTime time = new ElapsedTime();

        state = 0;
        timeReset = false;

        drive.resetEncoders();
        drive.resetAngle();
        time.reset();
        drive.update();

        telemetry.addData("state", state);
        telemetry.addData("current pos", drive.getCurrentPos());
        telemetry.addData("linear pos", drive.atLinearPos());
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        lift.init();
        arm.init();

        lift.resetEncoder();
        arm.resetEncoder();

        arm.runTo(90); // gets arm out of the intake's way

        intake.rightFullyOpen();

        time.reset();

        while(!isStopRequested()) {

            switch (state) {

                case 0:

                    arm.runTo(100); // gets arm out of the intake's way

                    intake.rightFullyOpen();

                    state++;

                    break;

                case 1:

                    if(lift.bottomSet){

                        state++;
                    }

                    else {

                        lift.tightenLiftString();

                        lift.goToBottom();

                        if (time.seconds() > 1) {
                            intake.leftFullyOpen();
                        }
                    }
                    break;

                case 2:

                    drive.goToPosition(-10, 0.25);

                    state++;
                    break;

                case 3:

                    if (drive.atLinearPos()){
                        drive.stopMotors();
                        state++;
                    }

                    break;

//                case 4:
//
//                    if (!timeReset){
//                        time.reset();
//                        timeReset = true;
//                    }
//
//                    if(time.seconds() < 2) {
//                        drive.strafeLeft();
//
//                    } else {
//
//                        state++;
//                    }
//
//                    break;
            }

            telemetry.addData("state", state);
            telemetry.addData("bottom set", lift.bottomSet);
//            telemetry.addData("current pos", drive.getCurrentPos());
//            telemetry.addData("target pos", drive.getTargetPos());
//            telemetry.addData("linear pos", drive.atLinearPos());

            telemetry.update();

        }
    }
}
