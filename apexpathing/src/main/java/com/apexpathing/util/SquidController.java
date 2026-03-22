package com.apexpathing.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A squID controller for follower
 * @author Xander Haemel - 31616
 */
public class SquidController {
    public double kP, kI, kD;
    public double goal = 0;
    private double integralSum = 0;
    private double derivative;
    private double lastError = 0;
    private double error;
    private double lastTimestamp = 0;
    private final ElapsedTime timer;

    public SquidController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer = new ElapsedTime();
        timer.startTime();
    }

    public void setGoal(double newGoal) {
        this.goal = newGoal;
    }

    /**
     * Calculates the PIDF output.
     * @param currentPosition the current position of the mechanism
     * @return the power output
     */
    public synchronized double calculate(double currentPosition) {
        double timestamp = timer.milliseconds() / 1000;
        double deltaTime = timestamp - lastTimestamp;

        if (deltaTime == 0) return 0;

        //p term, but its special because its a square root
        error = goal - currentPosition;
        //absolute value in case the error is negative, math.signum ensures the robot goes the right way
        double kPOut = kP * Math.sqrt(Math.abs(error)) * Math.signum(error);

        //i term, standard PID calculations
        integralSum += error * deltaTime;
        double iLimit = 0.25;
        if (integralSum > iLimit) integralSum = iLimit;
        if (integralSum < -iLimit) integralSum = -iLimit;
        double kIOut = kI * integralSum;

        derivative = (error - lastError) / deltaTime;
        double kDOut = kD * derivative;


        lastTimestamp = timestamp;
        lastError = error;

        return kPOut + kIOut + kDOut ;
    }
}
