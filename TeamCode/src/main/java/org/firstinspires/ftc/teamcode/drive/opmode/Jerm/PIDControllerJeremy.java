package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControllerJeremy {
    private double kP, kI, kD;
    private double targetPos;
    private double errorSum, lastError;
    private ElapsedTime timer;

    public PIDControllerJeremy(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.timer = new ElapsedTime();
    }

    public void setTargetPos(double targetPos) {
        this.targetPos = targetPos;
        errorSum = 0;
        lastError = 0;
        timer.reset();
    }

    public double calculate(double input) {
        double error = targetPos - input;
        double deltaTime = timer.seconds();
        double derivative = (error - lastError) / deltaTime;
        errorSum += error * deltaTime;
        double output = kP * error + kI * errorSum + kD * derivative;
        lastError = error;
        timer.reset();
        return Math.max(0.0, Math.min(1.0, output));
    }
}

