package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_MIDHIGHJUNCTION;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled

@TeleOp
public class dr4btest extends OpMode {
    DcMotorEx motorDR4B1, motorDR4B2;
    public double dr4bPower = 1;
    @Override
    public void init() {
        motorDR4B1 = hardwareMap.get(DcMotorEx.class, "Motor DR4B 1");
        motorDR4B2 = hardwareMap.get(DcMotorEx.class, "Motor DR4B 2");
        motorDR4B1.setDirection(DcMotorEx.Direction.REVERSE);
        motorDR4B2.setDirection(DcMotorEx.Direction.REVERSE);
        motorDR4B1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDR4B2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDR4B1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDR4B2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        setLiftPosition(DR4B_MIDHIGHJUNCTION);
    }
    public void setLiftPosition(int position){
        motorDR4B1.setTargetPosition(position);
        motorDR4B2.setTargetPosition(position);
        if (Math.abs(motorDR4B1.getCurrentPosition() - motorDR4B1.getTargetPosition()) < 7){
            motorDR4B1.setPower(0);
            motorDR4B2.setPower(0);
        } else {
            motorDR4B1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDR4B2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDR4B1.setPower(dr4bPower);
            motorDR4B2.setPower(dr4bPower);
        }
    }
}
