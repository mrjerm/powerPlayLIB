package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled

@TeleOp
public class dr4bencodertest extends OpMode {
    public DcMotorEx motorDR4B;


    @Override
    public void init() {
        motorDR4B = hardwareMap.get(DcMotorEx.class, "Motor DR4B");
        motorDR4B.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorDR4B.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("dr4b position", motorDR4B.getCurrentPosition());
        telemetry.update();
    }
}
