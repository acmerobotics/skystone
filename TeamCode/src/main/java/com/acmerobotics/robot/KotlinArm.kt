package com.acmerobotics.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.*

/*

This is a (simplified) copy of the armEncoder class but in Kotlin. I was curious about the
language and decided to give it a try.

*/

@Config
class KotlinArm constructor( // class is followed by the primary constructor
        // constructor scope

        // constructor parameter
        hardwareMap: HardwareMap // follows format of > parameterName: dataType = value
        // NOTE: it looks like the parameter, hardwareMap, is accessible outside the constructors scope
        // but only in the base class scope (so only accessible by class variables) not in any deeper
        // scopes for example the parameter can't be accessed in other functions (unless you save the parameter argument
        // as a property)

){

    // class scope

    // properties (instance fields in Java)

    // visiablility is public by default
    var armMotor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "armMotor") // DcMotorEx.class (java) = DcMotorEx::class.java (kotilin)
    // .java is used because the referenced class is in java not kotlin
    var handServo: Servo = hardwareMap.servo.get("handServo") // another way to instantiate a hardware device

    var position: Double = 0.0

    private val handOpenPos: Double = 0.59
    private val handClosePos: Double = 0.1 // val is like a final variable, once it is given
                                            // a value  it can be chande, it is a view only variable

    // static variables
    companion object{
        var P: Double = 20.0
        var I: Double = 0.25
        var D: Double = 0.0
        var F: Double = 0.0

        var coefficients: PIDFCoefficients = PIDFCoefficients(P, I, D, F)

    }

    // functions
    fun init(){
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD)

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)

        armMotor.setPower(0.0)

        armMotor.setTargetPosition(0)
    }


    fun resetEncoder(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    }


    fun setPID(pidfCoefficients: PIDFCoefficients){
        if (pidfCoefficients != coefficients){
            armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefficients)
            coefficients = pidfCoefficients
        }
    }


    fun runTo(position: Int){
        setPID(coefficients)

    }


    fun setHand(position: String){
        if (position.equals("open")){
            handServo.setPosition(handOpenPos)
        }
        else{
            handServo.setPosition(handClosePos)
        }
    }


    // just a function to demonstrate how to return a value
    fun atPosition(position: Double): Boolean{ // return data Type is Boolean
        if (position == this.position){
            return true
        }
        else{
            return false
        }
    }

}