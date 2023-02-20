package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_GROUNDFLOORTURRETCLEARANCE;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_LOWJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_LOWPOWER;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_MIDHIGHJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.EAST;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.NORTH;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.SOUTH1;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.SOUTH2;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_CONE1;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_CONE2;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_CONE3;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_CONE4;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_CONE5;
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled

@TeleOp

public class JudgeTeleOp extends OpMode {
    /*TODO: V4B AUTOLIFT WHEN TURNING TURRET, SEPARATE STATE MACHINE*/

    public DcMotorEx motorFL, motorBL, motorFR, motorBR;
    public DcMotorEx motorDR4B;
    public DcMotorEx motorTurret;
    public DcMotor lightLeft, lightRight;


    public DcMotorEx underglow;
    public DcMotorEx grabberLight;

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
    public boolean stackPrevious = false;
    public boolean scoringStack = false;




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
        HORIZONTAL,
        STACK1,
        STACK2,
        STACK3,
        STACK4,
        STACK5;
    }

    V4BState v4BState = V4BState.RETRACTED;

    private enum RobotState{
        PICKING_UP,
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        RETRACT,
        STACK1,
        STACK2,
        STACK3,
        STACK4,
        STACK5;
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

        public RobotState stackUp(){
            switch (this){
                case STACK5: return STACK4;
                case STACK4: return STACK3;
                case STACK3: return STACK2;
                case STACK2: return STACK1;
                case STACK1: return STACK1;
                default: return STACK1;
            }
        }
        public RobotState stackDown(){
            switch (this){
                case STACK1: return STACK2;
                case STACK2: return STACK3;
                case STACK3: return STACK4;
                case STACK4: return STACK5;
                case STACK5: return STACK5;
                default: return STACK1;
            }
        }
    }

    RobotState robotState = RobotState.LOW_JUNCTION;

    RobotState previousRobotState;

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

        motorDR4B = hardwareMap.get(DcMotorEx.class, "Motor DR4B");
        motorDR4B.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorDR4B.setDirection(DcMotorEx.Direction.REVERSE);

        motorTurret = hardwareMap.get(DcMotorEx.class, "Motor Turret");

        underglow = hardwareMap.get(DcMotorEx.class, "Underglow");
        grabberLight = hardwareMap.get(DcMotorEx.class, "Grabber Light");

        servoGrabber = hardwareMap.get(Servo.class, "Servo Intake");
        servoV4BL = hardwareMap.get(Servo.class, "Servo V4BL");
        servoV4BR = hardwareMap.get(Servo.class, "Servo V4BR");
        servoV4BL.setDirection(Servo.Direction.REVERSE);

        lightLeft = hardwareMap.get(DcMotor.class, "Light Left");
        lightRight = hardwareMap.get(DcMotor.class, "Light Right");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");

        magLimSwitch = hardwareMap.get(TouchSensor.class, "Magnetic Limit Switch");
    }

    @Override
    public void loop() {
//        turtle(gamepad1.y, gamepad1.a);
        if (pizza) {
            resetRuntime();
        }
        pizza = false;
        /*TODO: time this to the Turret slide, only for 5s*/
        if (getRuntime() > 75){
            flashing = true;
        }
        if (flashing && Math.round(getRuntime()*8)/8f % 0.25 == 0){
            lightLeft.setPower(1);
            lightRight.setPower(1);
        } else if (flashing){
            lightLeft.setPower(0);
            lightRight.setPower(0);
        }
        spinny(gamepad1.left_bumper, gamepad1.right_bumper);
        grippers(gamepad1.left_trigger > 0.3, gamepad1.right_trigger > 0.3);
        setRobotState(gamepad1.dpad_up, gamepad1.dpad_down);
        liftControl();
        v4bControl();
        low(gamepad1.a);
        highLeft(gamepad1.x);
        highRight(gamepad1.b);
        junctionFinder();
        resetTurret(gamepad1.dpad_left, gamepad1.dpad_right);

/*        lift(gamepad2.dpad_up, gamepad2.dpad_down);
        stick(gamepad2.y, gamepad2.a);*/
    }

     public void stackControl(boolean keybind){
        boolean stackCurrent = keybind;
        if (stackCurrent && !stackPrevious){
            if (scoringStack){
                robotState = previousRobotState;
                scoringStack = false;
            } else {
                previousRobotState = robotState;
                robotState = RobotState.STACK1;
                scoringStack = true;
            }
        }
        stackPrevious = stackCurrent;
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
            grabberLight.setPower(-1);
        }
        else {
            grabberLight.setPower(0);
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
        if (!scoringStack) {
            boolean upCurrent = up;
            if (upCurrent && !upPrevious) {
                robotState = robotState.next();
                dr4bPower = 1;
            }
            upPrevious = upCurrent;

            boolean downCurrent = down;
            if (downCurrent && !downPrevious) {
                robotState = robotState.previous();
                dr4bPower = DR4B_LOWPOWER;
            }
            downPrevious = downCurrent;
        } else {
            boolean upCurrent = up;
            if (upCurrent && !upPrevious) {
                robotState = robotState.stackUp();
                dr4bPower = 1;
            }
            upPrevious = upCurrent;

            boolean downCurrent = down;
            if (downCurrent && !downPrevious) {
                robotState = robotState.stackDown();
                dr4bPower = DR4B_LOWPOWER;
            }
            downPrevious = downCurrent;
        }
    }


    public void liftControl(){
        if (!scoringStack) {
            switch (robotState) {
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
        } else {
            switch (robotState){
                case STACK1:
                    dr4BState = DR4BState.REST;
                    dr4bPower = DR4B_LOWPOWER;
                    break;
                case STACK2:
                    dr4BState = DR4BState.REST;
                    dr4bPower = DR4B_LOWPOWER;
                    break;
                case STACK3:
                    dr4BState = DR4BState.REST;
                    dr4bPower = DR4B_LOWPOWER;
                    break;
                case STACK4:
                    dr4BState = DR4BState.REST;
                    dr4bPower = DR4B_LOWPOWER;
                    break;
                case STACK5:
                    dr4BState = DR4BState.REST;
                    dr4bPower = DR4B_LOWPOWER;
                    break;
                default:
                    dr4BState = DR4BState.REST;
                    dr4bPower = DR4B_LOWPOWER;
                    break;
            }
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
        if (!scoringStack) {
            switch (robotState) {
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
        } else {
            switch (robotState) {
                case STACK1:
                    v4BState = V4BState.STACK1;
                    break;
                case STACK2:
                    v4BState = V4BState.STACK2;
                    break;
                case STACK3:
                    v4BState = V4BState.STACK3;
                    break;
                case STACK4:
                    v4BState = V4BState.STACK4;
                    break;
                case STACK5:
                    v4BState = V4BState.STACK5;
                    break;
                default:
                    v4BState = V4BState.STACK1;
                    break;
            }
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
            case STACK1:
                setV4B(V4B_CONE1);
                break;
            case STACK2:
                setV4B(V4B_CONE2);
                break;
            case STACK3:
                setV4B(V4B_CONE3);
                break;
            case STACK4:
                setV4B(V4B_CONE4);
                break;
            case STACK5:
                setV4B(V4B_CONE5);
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
        motorDR4B.setTargetPosition(position);
        if (Math.abs(motorDR4B.getCurrentPosition() - motorDR4B.getTargetPosition()) < 10){
            motorDR4B.setPower(0);
        } else {
            motorDR4B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDR4B.setPower(dr4bPower);
        }
    }

    public void setV4B(double position){
        servoV4BL.setPosition(position * V4B_SCALELEFT);
        servoV4BR.setPosition(position);
    }
}
