package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.BurlingameLift;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Lift Testing")
@Config
public class TestLift extends LinearOpMode {

    private boolean isUpDown = false;
    private boolean isDownDown = false;
    private boolean isLeftDown = false;
    private boolean isRightDown = false;

    @Override
    public void runOpMode() throws InterruptedException {

        BurlingameLift lift = new BurlingameLift(hardwareMap);

        lift.init();
        lift.resetEncoder();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while(!isStopRequested()){

            //lift.set(gamepad1.left_stick_y);

            if (gamepad2.dpad_up){
                isUpDown = true;

            } else if (isUpDown){
                lift.goToIntake();
                isUpDown = false;
            }

            if (gamepad2.dpad_down){
                isDownDown = true;

            } else if (isDownDown){
                lift.goToBottom();
                isDownDown = false;
            }

            if (gamepad2.dpad_right) {
                isRightDown = true;

            } else if (isRightDown) {
                lift.adjustLiftUp();
                isRightDown = false;
            }

            if (gamepad2.dpad_left) {
                isLeftDown = true;

            } else if (isLeftDown) {
                lift.adjustLiftDown();
                isLeftDown = false;
            }

            telemetry.addData("encoder pos", lift.checkEncoder());
            telemetry.update();

            dashboardTelemetry.addData("current position", lift.checkEncoder());
            dashboardTelemetry.addData("target position", lift.liftMotor.getTargetPosition());
            dashboardTelemetry.addData("pid", lift.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            dashboardTelemetry.update();

        }

    }
}
