package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.liftEncoder;
import com.acmerobotics.robot.Intake;

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
        Drive drive = new Drive(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        armEncoder arm = new armEncoder(hardwareMap);

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
        isRightOpen = true;

        time.reset();

        while(!isStopRequested()) {

            switch (state) {

                //TODO add the init sequence

                case 0:

                    lift.tightenLiftString();

                    if(time.seconds() > 1){
                        intake.leftFullyOpen();
                        isLeftOpen = true;
                        isFullyOpen = true;
                    }

                    lift.goToBottom();


                    if(lift.bottomSet){
                        break;
                    }

                case 1:

                    drive.goToPosition(10);

                    state++;

                    break;

                case 2:

                    if(drive.atLinearPos()){
                        state++;

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
