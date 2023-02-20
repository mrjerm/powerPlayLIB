package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class odometerTest extends OpMode {
    public DcMotorEx odometerX;
    public DcMotorEx odometerYL;
    public DcMotorEx odometerYR;
    @Override
    public void init() {
        odometerX = hardwareMap.get(DcMotorEx.class, "Motor FL");
        odometerYL = hardwareMap.get(DcMotorEx.class, "Motor BR");
        odometerYR = hardwareMap.get(DcMotorEx.class, "Grabber Light");
        odometerX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerYL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerYR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        telemetry.addData("Odometer x", odometerX.getCurrentPosition());
        telemetry.addData("Odometer yl", odometerYL.getCurrentPosition());
        telemetry.addData("Odometer yr", odometerYR.getCurrentPosition());
        telemetry.update();
    }
}
