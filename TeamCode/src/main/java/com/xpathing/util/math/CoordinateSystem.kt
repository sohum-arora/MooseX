package com.xpathing.util.math

interface CoordinateSystem {
    fun toApexCoordinates(pose: Pose): Pose
    fun fromApexCoordinates(pose: Pose): Pose
}