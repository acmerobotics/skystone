package com.acmerobotics.robot

//import com.acmerobotics.robot.KotlinArm
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

/*
An opMode written in Kotlin
 */

class Test2: LinearOpMode(){

    override fun runOpMode(){
        val arm = KotlinArm(hardwareMap)

        waitForStart()

        if (isStopRequested()){
            return
        }

        while(!isStopRequested){
            if (gamepad1.x) {
                arm.runTo(100) // using a kotlin function
           }

           if (gamepad1.a) {
                arm.armMotor.setPower(0.5) // using a kotlin function by first accessing a kotlin
                                            // property that has the annotation @JvmField so it armMotor
                                            // can be treated as a java instance field
            }

            if (gamepad1.b){
                arm.armMotor.setPower(arm.armPower) // get the armPower using a getter
            }

            telemetry.addData("power", arm.armMotor.getPower())
            telemetry.addLine();
            telemetry.addData("target position", arm.armMotor.getTargetPosition())
            telemetry.addData("current position", arm.armMotor.getCurrentPosition())
            telemetry.update()
        }
    }
}