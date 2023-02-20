package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;
@Disabled

@TeleOp
public class EncoderTest extends OpMode {

    public DcMotorEx revEncoder;

    @Override
    public void init() {
        revEncoder = hardwareMap.get(DcMotorEx.class, "Motor DR4B 1");
        revEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            revEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        telemetry.addData("position", revEncoder.getCurrentPosition());
        telemetry.update();
    }
}
