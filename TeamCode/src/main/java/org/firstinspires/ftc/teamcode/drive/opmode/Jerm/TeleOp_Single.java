package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_GROUNDFLOORTURRETCLEARANCE;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_LOWJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_LOWPOWER;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_MIDHIGHJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.EAST;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.NORTH;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.SOUTH1;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.SOUTH2;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_FLOOR;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_GROUNDJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_HIGHJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_LOWMID;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_SCALELEFT;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_TURRETCLEARANCE;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_VERTICAL;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.WEST;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.grabberClose;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.grabberOpen;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.max;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.min;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.speedLimit;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.turretDefaultPower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class TeleOp_Single extends OpMode {
    /*TODO: V4B AUTOLIFT WHEN TURNING TURRET, SEPARATE STATE MACHINE*/

    public DcMotorEx motorFL, motorBL, motorFR, motorBR;
    public DcMotorEx motorDR4B1;
    public DcMotorEx motorDR4B2;
    public DcMotorEx motorTurret;
    DcMotorEx revEncoder;

    public DigitalChannel grabberLight;

    public DcMotorEx underglow;

    public Servo servoGrabber;
    public Servo servoV4BL, servoV4BR;

    public DistanceSensor distanceSensor;

    public TouchSensor magLimSwitch;

//    public CRServo servoIntake;
//    public Servo servoV4B;



    public boolean turretLeftPrevious = false;
    public boolean turretRightPrevious = false;
    public boolean moveUpPrevious = false;
    public boolean moveDownPrevious = false;
    public boolean extendPrevious = false;
    public boolean retractPrevious = false;
    public boolean upPrevious = false;
    public boolean downPrevious = false;
    public double dr4bPower = 1;
    public boolean pizza = true;
    public boolean flashing = false;



    private enum TurretState{
        SOUTH1,
        WEST,
        NORTH,
        EAST,
        SOUTH2;
        public TurretState next(){
            switch (this){
                case SOUTH1: return WEST;
                case WEST: return NORTH;
                case NORTH: return EAST;
                case EAST: return SOUTH2;
                case SOUTH2: return SOUTH2;
                default: return NORTH;
            }
        }
        public TurretState previous(){
            switch (this){
                case SOUTH2: return EAST;
                case EAST: return NORTH;
                case NORTH: return WEST;
                case WEST: return SOUTH1;
                case SOUTH1: return SOUTH1;
                default: return NORTH;
            }
        }
    }
    TurretState turretState = TurretState.NORTH;


    private enum DR4BState{
        REST,
        LOW,
        MID,
        HIGH;
        public DR4BState next(){
            switch (this){
                case REST: return LOW;
                case LOW: return MID;
                case MID: return HIGH;
                case HIGH: return HIGH;
                default: return REST;
            }
        }
        public DR4BState previous(){
            switch (this){
                case HIGH: return MID;
                case MID: return LOW;
                case LOW: return REST;
                case REST: return REST;
                default: return REST;
            }
        }
    }

    DR4BState dr4BState = DR4BState.REST;

    private enum V4BState{
        RETRACTED,
        HIGH,
        MID,
        LOW,
        GROUND,
        FLOOR,
        TURRETCLEARANCE,
        HORIZONTAL;

    }

    V4BState v4BState = V4BState.RETRACTED;

    private enum RobotState{
        PICKING_UP,
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        RETRACT;
        public RobotState next(){
            switch (this){
                case RETRACT: return PICKING_UP;
                case PICKING_UP: return GROUND_JUNCTION;
                case GROUND_JUNCTION: return LOW_JUNCTION;
                case LOW_JUNCTION: return MEDIUM_JUNCTION;
                case MEDIUM_JUNCTION: return HIGH_JUNCTION;
                case HIGH_JUNCTION: return HIGH_JUNCTION;
                default: return RETRACT;
            }
        }
        public RobotState previous(){
            switch (this){
                case HIGH_JUNCTION: return MEDIUM_JUNCTION;
                case MEDIUM_JUNCTION: return LOW_JUNCTION;
                case LOW_JUNCTION: return GROUND_JUNCTION;
                case GROUND_JUNCTION: return PICKING_UP;
                case PICKING_UP: return RETRACT;
                case RETRACT: return RETRACT;
                default: return RETRACT;
            }
        }
    }

    RobotState robotState = RobotState.LOW_JUNCTION;

    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotorEx.class, "Motor FL");
        motorBL = hardwareMap.get(DcMotorEx.class, "Motor BL");
        motorFR = hardwareMap.get(DcMotorEx.class, "Motor FR");
        motorBR = hardwareMap.get(DcMotorEx.class, "Motor BR");
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorDR4B1 = hardwareMap.get(DcMotorEx.class, "Motor DR4B 1");
        motorDR4B2 = hardwareMap.get(DcMotorEx.class, "Motor DR4B 2");
        motorDR4B1.setDirection(DcMotorEx.Direction.REVERSE);
        motorDR4B2.setDirection(DcMotorEx.Direction.REVERSE);
        revEncoder = hardwareMap.get(DcMotorEx.class, "Motor DR4B 1");
        revEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        revEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDR4B1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDR4B2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDR4B1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDR4B2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        motorTurret = hardwareMap.get(DcMotorEx.class, "Motor Turret");

        underglow = hardwareMap.get(DcMotorEx.class, "Underglow");

        servoGrabber = hardwareMap.get(Servo.class, "Servo Intake");
        servoV4BL = hardwareMap.get(Servo.class, "Servo V4BL");
        servoV4BR = hardwareMap.get(Servo.class, "Servo V4BR");
        servoV4BL.setDirection(Servo.Direction.REVERSE);

        grabberLight = hardwareMap.get(DigitalChannel.class, "Grabber Light");
        grabberLight.setMode(DigitalChannel.Mode.OUTPUT);
        grabberLight.setState(false);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");

        magLimSwitch = hardwareMap.get(TouchSensor.class, "Magnetic Limit Switch");
    }

    @Override
    public void loop() {
        turtle(gamepad1.dpad_left, gamepad1.dpad_right);
        if (pizza) {
            resetRuntime();
        }
        pizza = false;
        if (getRuntime() > 75){
            flashing = true;
        }
        if (flashing && Math.round(getRuntime()*8)/8f % 0.25 == 0){
            underglow.setPower(-1);
        } else if (flashing){
            underglow.setPower(0);
        }
        drive();
        spinny(gamepad1.left_bumper, gamepad1.right_bumper);
        grippers(gamepad1.left_trigger > 0.3, gamepad1.right_trigger > 0.3);
        setRobotState(gamepad1.dpad_up, gamepad1.dpad_down);
        liftControl();
        v4bControl();
        low(gamepad1.a);
        highLeft(gamepad1.x);
        highRight(gamepad1.b);
        junctionFinder();
        resetTurret(gamepad1.left_stick_button, gamepad1.right_stick_button);



/*        lift(gamepad2.dpad_up, gamepad2.dpad_down);
        stick(gamepad2.y, gamepad2.a);*/
    }

    public void resetTurret(boolean keybindLeft, boolean keybindRight){
        if (keybindLeft){
            motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorTurret.setPower(-0.7);
            while (!magLimSwitch.isPressed()){

            }
            motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorTurret.setPower(0);
        }
        if (keybindRight){
            motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorTurret.setPower(0.7);
            while (!magLimSwitch.isPressed()){

            }
            motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorTurret.setPower(0);
        }
    }

    public void junctionFinder(){
        if (distanceSensor.getDistance(DistanceUnit.MM) < 400){
            grabberLight.setState(true);
        }
        else {
            grabberLight.setState(false);
        }
    }

    public void highLeft(boolean keybind){
        if (keybind){
            robotState = RobotState.HIGH_JUNCTION;
            dr4bPower = 1;
            turretState = TurretState.EAST;
        }
    }

    public void highRight(boolean keybind){
        if (keybind){
            robotState = RobotState.HIGH_JUNCTION;
            dr4bPower = 1;
            turretState = TurretState.WEST;
        }
    }

    public void low(boolean keybind){
        if (keybind) {
            if (turretState == TurretState.EAST){
                turretState = TurretState.NORTH;
            } else if (turretState == TurretState.WEST){
                turretState = TurretState.NORTH;
            }
            switch (turretState) {
                case SOUTH1:
                    setTurretPosition(SOUTH1);
                    break;
                case SOUTH2:
                    setTurretPosition(SOUTH2);
                    break;
                case EAST:
                    setTurretPosition(EAST);
                    break;
                case NORTH:
                    setTurretPosition(NORTH);
                    break;
                case WEST:
                    setTurretPosition(WEST);
                    break;
                default:
                    telemetry.addData("turret status", "we messed up 💀");
                    telemetry.update();
            }
            robotState = RobotState.PICKING_UP;
            dr4bPower = DR4B_LOWPOWER;

        }
    }

    public void setRobotState(boolean up, boolean down){
        boolean upCurrent = up;
        if (upCurrent && !upPrevious){
            robotState = robotState.next();
            dr4bPower = 1;
        }
        upPrevious = upCurrent;

        boolean downCurrent = down;
        if (downCurrent && !downPrevious){
            robotState = robotState.previous();
            dr4bPower = DR4B_LOWPOWER;
        }
        downPrevious = downCurrent;
    }

    public void liftControl(){
        switch (robotState){
            case PICKING_UP:
                dr4BState = DR4BState.REST;
                break;
            case GROUND_JUNCTION:
                dr4BState = DR4BState.REST;
                break;
            case LOW_JUNCTION:
                dr4BState = DR4BState.LOW;
                break;
            case MEDIUM_JUNCTION:
                dr4BState = DR4BState.MID;
                break;
            case HIGH_JUNCTION:
                dr4BState = DR4BState.HIGH;
                break;
            case RETRACT:
                dr4BState = DR4BState.REST;
                break;
            default:
                dr4BState = DR4BState.REST;
        }

        switch (dr4BState){
            case REST:
                setLiftPosition(DR4B_GROUNDFLOORTURRETCLEARANCE);
                break;
            case LOW:
                setLiftPosition(DR4B_LOWJUNCTION);
                break;
            case MID:
                setLiftPosition(DR4B_MIDHIGHJUNCTION);
                break;
            case HIGH:
                setLiftPosition(DR4B_MIDHIGHJUNCTION);
                break;
            default:
                setLiftPosition(DR4B_GROUNDFLOORTURRETCLEARANCE);
        }
    }

    public void v4bControl(){
        switch (robotState){
            case PICKING_UP:
                v4BState = V4BState.FLOOR;
                break;
            case GROUND_JUNCTION:
                v4BState = V4BState.GROUND;
                break;
            case LOW_JUNCTION:
                v4BState = V4BState.LOW;
                break;
            case MEDIUM_JUNCTION:
                v4BState = V4BState.MID;
                break;
            case HIGH_JUNCTION:
                v4BState = V4BState.HIGH;
                break;
            case RETRACT:
                v4BState = V4BState.RETRACTED;
                break;
            default:
                v4BState = V4BState.RETRACTED;
        }

        switch (v4BState){
            case RETRACTED:
                setV4B(V4B_VERTICAL);
                servoGrabber.setPosition(grabberClose);
                break;
            case HIGH:
                setV4B(V4B_HIGHJUNCTION);
                break;
            case MID:
                setV4B(V4B_LOWMID);
                break;
            case LOW:
                setV4B(V4B_LOWMID);
                break;
            case GROUND:
                setV4B(V4B_GROUNDJUNCTION);
                break;
            case FLOOR:
                setV4B(V4B_FLOOR);
                break;
            case TURRETCLEARANCE:
                setV4B(V4B_TURRETCLEARANCE);
                break;
            case HORIZONTAL:
                setV4B(V4B_HORIZONTAL);
                break;
            default:
                setV4B(V4B_VERTICAL);
        }
    }

    public void turtle(boolean fast, boolean slow){
        if (fast){
            speedLimit = max;
        }
        else if (slow){
            speedLimit = min;
        }
    }

    public void drive() {
        float x1 = -gamepad1.left_stick_x;
        float y1 =  gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;

        double fl = x1 + y1 + x2;
        double bl = -x1 + y1 + x2;
        double fr = -x1 + y1 - x2;
        double br = x1 + y1 - x2;

        motorFL.setPower(fl * speedLimit);
        motorBL.setPower(bl * speedLimit);
        motorFR.setPower(fr * speedLimit);
        motorBR.setPower(br * speedLimit);
    }

    public void spinny(boolean left, boolean right){
        if (robotState != RobotState.PICKING_UP) {
            boolean turretLeftCurrent = left;
            if (turretLeftCurrent && !turretLeftPrevious) {
                turretState = turretState.previous();
            }
            turretLeftPrevious = turretLeftCurrent;

            boolean turretRightCurrent = right;
            if (turretRightCurrent && !turretRightPrevious) {
                turretState = turretState.next();
            }
            turretRightPrevious = turretRightCurrent;

            switch (turretState) {
                case SOUTH1:
                    setTurretPosition(SOUTH1);
                    break;
                case SOUTH2:
                    setTurretPosition(SOUTH2);
                    break;
                case EAST:
                    setTurretPosition(EAST);
                    break;
                case NORTH:
                    setTurretPosition(NORTH);
                    break;
                case WEST:
                    setTurretPosition(WEST);
                    break;
                default:
                    telemetry.addData("turret status", "we messed up 💀");
                    telemetry.update();
            }
        }
        telemetry.addData("turret state", turretState);
        telemetry.update();
    }

    public void setTurretPosition(int position){
        motorTurret.setTargetPosition(position);
        motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(turretDefaultPower);
    }

    public void grippers(boolean open, boolean close){
        if (open){
            servoGrabber.setPosition(grabberOpen);
        }
        if (close){
            servoGrabber.setPosition(grabberClose);
        }
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

    public void setV4B(double position){
        servoV4BL.setPosition(position * V4B_SCALELEFT);
        servoV4BR.setPosition(position);
    }
}
