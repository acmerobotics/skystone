package com.acmerobotics.opmodes.AutoOpModes;


import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
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
        Drive drive = new Drive(hardwareMap);
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

        while(!isStopRequested()) {

            switch (state) {

                //TODO add the init sequence

                case 0:

                    drive.goToPosition(10);

                    state++;

                    break;

                case 1:

                    if(drive.atLinearPos()){
                        drive.resetLinearPos();
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
