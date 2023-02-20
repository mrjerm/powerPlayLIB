package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.turretMaxPower;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.turretMinPower;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@Config
@Disabled

@TeleOp
public class pidtest extends LinearOpMode {
    DcMotorEx motor1, motor2;
    DcMotorEx revEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "Motor DR4B 1");
        motor2 = hardwareMap.get(DcMotorEx.class, "Motor DR4B 2");
        motor1.setDirection(DcMotorEx.Direction.REVERSE);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);
        revEncoder = hardwareMap.get(DcMotorEx.class, "Motor DR4B 1");
        revEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        revEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        int targetPosition = 1000;

        while (opModeIsActive()) {
            move(targetPosition);
            while (!isStopRequested()){
                motor1.setPower(0);
                motor2.setPower(0);
            }
        }
    }

//    public void move(int targetTicks) {
//        int distance = Math.abs(revEncoder.getCurrentPosition() - targetTicks);
//        while (Math.abs(revEncoder.getCurrentPosition() - targetTicks) > 10) {
////            double power = Range.clip(((-1 * (Math.pow(((Math.abs(targetTicks - revEncoder.getCurrentPosition()) / (distance / 2)) - 1), 6))) + 1), 0.3, 1.0);
//            double power = Range.clip(((-1 * (Math.pow(((Math.abs(targetTicks - revEncoder.getCurrentPosition()) / (distance / 2)) - 1), 6))) + 1), 0.3, 1.0);
//            motor1.setPower(power);
//            motor2.setPower(power);
//            // Telemetry the theoretical power
//            telemetry.addData("power", power);
//            telemetry.update();
//        }
//        telemetry.addData("Done", "");
//        telemetry.update();
//        motor1.setPower(0);
//        motor2.setPower(0);
//    }
    public void move(int targetTicks) {
        while (Math.abs(revEncoder.getCurrentPosition() - targetTicks) > 10) {
//            double power = Range.clip(((-1 * (Math.pow(((Math.abs(targetTicks - revEncoder.getCurrentPosition()) / (distance / 2)) - 1), 6))) + 1), 0.3, 1.0);
            double power = 1;
            motor1.setPower(power);
            motor2.setPower(power);
            // Telemetry the theoretical power
            telemetry.addData("power", power);
            telemetry.update();
        }
        telemetry.addData("Done", "");
        telemetry.update();
        motor1.setPower(0);
        motor2.setPower(0);
    }

}