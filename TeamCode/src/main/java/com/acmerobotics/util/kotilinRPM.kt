package com.acmerobotics.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime

class kotlinRPM constructor(
        hardwareMap: HardwareMap
){

    @JvmField var motor: DcMotor = hardwareMap.get(DcMotor::class.java, "motor")
    var time: ElapsedTime = ElapsedTime()

    var TICKS_PER_REVOLUTION: Int = 0
    var accumatedTicks: Int = 0


    fun getTicks(): Int{
        return motor.getCurrentPosition()
    }


    fun ticksPerSec(): Int{
        time.reset()

        var totalTicks: Int = 0

        while (time.seconds() <= 1){
            totalTicks = getTicks() - accumatedTicks
        }

        accumatedTicks += totalTicks

        return totalTicks
    }

    fun getRPM(): Double{
        var ticks: Int = ticksPerSec() * 60

        var RPM: Double = (ticks/ TICKS_PER_REVOLUTION).toDouble()

        return RPM
    }
}