package com.apexpathing.localization

import com.apexpathing.util.math.ApexCoordinates
import com.apexpathing.util.math.Pose
import com.apexpathing.util.math.Vector

class PoseTracker
    constructor(
        localizer: Localizer
    )
{
    init {
        localizer.resetIMU()
    }
    @get:JvmName("startPose")
    @set:JvmName("setStartPose")
    var startPose: Pose = Pose(0.0,0.0,0.0, ApexCoordinates)

    @get:JvmName("prevPose")
    var prevPose: Pose = startPose.copy()
        private set
    @get:JvmName("currPose")
    var currPose: Pose = startPose.copy()
        private set

    @get:JvmName("prevVelocity")
    var prevVelocity: Vector = Vector()
        private set
    @get:JvmName("currVelocity")
    var currVelocity: Vector = Vector()
        private set

    @get:JvmName("prevAcceleration")
    var prevAcceleration: Vector = Vector()
        private set
    @get:JvmName("currAcceleration")
    var currAcceleration: Vector = Vector()
        private set

    @get:JvmName("xOffset")
    @set:JvmName("yOffset")
    var xOffset: Double = 0.0
    @get:JvmName("yOffset")
    @set:JvmName("yOffset")
    var yOffset: Double = 0.0

    @get:JvmName("prevPoseTimeStamp")
    var prevPoseTimeStamp: Long = 0
        private set
    @get:JvmName("currPoseTimeStamp")
    var currPoseTimeStamp: Long = 0
        private set

}

interface Localizer {
    fun resetIMU()
}