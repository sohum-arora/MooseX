package com.xpathing.util.math

<<<<<<< HEAD
import com.xpathing.main.follower.paths.Pose
=======
>>>>>>> 76fcf68cfb766210971a5f09e054adc7cb309329
import kotlin.math.PI

class Pose(var x: Double, var y: Double, var theta:Double = 0.0, var z: Double = 0.0) {
    /**
    *@param at A [Double] that we can flip the pose over (y = at)
    *@return A [Pose] that contains the reflection of the pose over the x-axis parallel line
    */
    fun reflectX(at: Double): Pose {
        y = 2 * at - y
        theta = normalize(-theta)
        return this
    }

    /**
    *@param at A [Double] that we can flip the pose over (x = at)
    *@return A [Pose] that contains the reflection of the pose over the y-axis parallel line
    */
    fun reflectY(at: Double): Pose {
        x = 2 * at - x
        theta = normalize(PI - theta)
        return this
    }

    /**
    *@param at A [Double] offset for the line y = x + at
    *@return A [Pose] that contains the reflection of the pose over the line y = x + at
    */
    fun reflectYX(at: Double): Pose { 
        val oldX = x
        x = y - at
        y = oldX + at
        theta = normalize(PI / 2.0 - theta)
        return this
    }
<<<<<<< HEAD
    fun addPose(other: com.xpathing.main.follower.paths.Pose): com.xpathing.main.follower.paths.Pose {
        return Pose(
            x + other.x,
            y + other.y,
            Pose.Companion.normalize(heading + other.heading)
        )
    }
    fun rotate(angle : Double) : Pose {
        val cosA = Math.cos(angle)
        val sinA = Math.sin(angle)

        return Pose(
            x * cosA - y * sinA,
            x * sinA + y * cosA,
            Pose.Companion.normalize(heading + angle)
        )
    }
    fun subtract(other: Pose): Pose {
        return Pose(
            x - other.x,
            y - other.y,
            Pose.normalize(heading - other.heading)
        )
    }
    fun getDistance(other : Pose) : Double {
        val dX = other.x - x
        val dY = other.y - y
        return Math.hypot(dX, dY)
    }
    fun scale(k: Double): Pose {
        return Pose(x * k, y * k, heading * k)
    }
    fun copyPose(): Pose {
        return Pose(x, y, heading)
    }

   companion object { fun normalize(angle: Double): Double {
=======

    fun normalize(angle: Double): Double {
>>>>>>> 76fcf68cfb766210971a5f09e054adc7cb309329
        var a = angle % (2 * PI)
        if (a > PI) a -= 2 * PI
        if (a <= -PI) a += 2 * PI
        return a
    }
<<<<<<< HEAD
   }
=======
>>>>>>> 76fcf68cfb766210971a5f09e054adc7cb309329
}   