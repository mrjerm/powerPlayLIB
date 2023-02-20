package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_GROUNDFLOORTURRETCLEARANCE;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_LOWJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_LOWPOWER;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_MIDHIGHJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.EAST;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.NORTH;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_HIGHJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_LOWMID;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_RETRACTED;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_SCALELEFT;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_VERTICAL;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.WEST;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.autoSouthEast;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.autoSouthWest;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.grabberClose;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.grabberOpen;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.starterStack;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.turretDefaultPower;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.turretMaxPower;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.turretMinPower;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Left_5 extends LinearOpMode {

    static double timeStamp;

    public DcMotorEx motorDR4B1;
    public DcMotorEx motorDR4B2;

    public DcMotorEx motorTurret;

    public Servo servoGrabber;
    public Servo servoV4BL, servoV4BR;

    public DigitalChannel grabberLight;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    private TrajectorySequence trajFinal;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        //vision
        //servo initialization

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        motorDR4B1 = hardwareMap.get(DcMotorEx.class, "Motor DR4B 1");
        motorDR4B1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorDR4B1.setDirection(DcMotorEx.Direction.REVERSE);
        motorDR4B1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorDR4B2 = hardwareMap.get(DcMotorEx.class, "Motor DR4B 2");
        motorDR4B2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorDR4B2.setDirection(DcMotorEx.Direction.REVERSE);
        motorDR4B2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorTurret = hardwareMap.get(DcMotorEx.class, "Motor Turret");
        motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servoGrabber = hardwareMap.get(Servo.class, "Servo Intake");
        servoGrabber.setPosition(grabberClose);
        servoV4BL = hardwareMap.get(Servo.class, "Servo V4BL");
        servoV4BR = hardwareMap.get(Servo.class, "Servo V4BR");
        servoV4BL.setDirection(Servo.Direction.REVERSE);

        grabberLight = hardwareMap.get(DigitalChannel.class, "Grabber Light");
        grabberLight.setMode(DigitalChannel.Mode.OUTPUT);
        grabberLight.setState(false);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    setTurretPosition(EAST);
                })
                .lineToLinearHeading(new Pose2d(-38, -22, Math.toRadians(88)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    prepareStack(1); //prepare v4b + dr4b for starter stack cone 1
                })
                .lineToLinearHeading(new Pose2d(-38, -12, Math.toRadians(88)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .lineToLinearHeading(new Pose2d(-54, -6.5, Math.toRadians(-186)))
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .forward(0.4)
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .forward(0.4)
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .forward(0.5)
                .build();

        servoV4BL.setPosition(V4B_RETRACTED);
        servoV4BR.setPosition(V4B_RETRACTED);
        setTurretPosition(NORTH);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }

            }

            telemetry.update();
            sleep(20);


            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                trajFinal = drive.trajectorySequenceBuilder(traj6.end())
                        .forward(1.4)
                        .build();
            } else if (tagOfInterest.id == MIDDLE) {
                trajFinal = drive.trajectorySequenceBuilder(traj6.end())
                        .back(19.6)
                        .build();
            } else {
                trajFinal = drive.trajectorySequenceBuilder(traj6.end())
                        .back(42.6)
                        .build();
            }
        }

        if (opModeIsActive()){
            //prepare to drop off preload
            setMid();
            //move to (4, 2) junction
            drive.followTrajectorySequence(traj1);
            //drop dr4b to align cone on junction
            setLift(DR4B_MIDHIGHJUNCTION - 75);
            waitForLift();
            //score preload cone
            openGrabber();

            //retract v4b to prepare to move to starter stack
            setV4B(V4B_VERTICAL);
            pause(0.1);
            //point turret towards starter stack
            setTurretPosition(NORTH);
            waitForTurret(turretMinPower, turretMaxPower);
            //pause(0.1);

            //move to starter stack
            drive.followTrajectorySequence(traj2);
            drive.turn(Math.toRadians(90));
            drive.followTrajectorySequence(traj3);

            //grab cone 1
            closeGrabber();
            sleep(200);
            setLift(DR4B_LOWJUNCTION);
            waitForLift();
            setV4B(V4B_HORIZONTAL);
            setTurretPosition(autoSouthWest+20);
            waitForTurret(turretMinPower, turretMaxPower);
            setLift(DR4B_LOWJUNCTION - 75);
            waitForLift();
            openGrabber();
            setV4B(V4B_VERTICAL);
            sleep(250);
            setTurretPosition(NORTH);
            waitForTurret(turretMinPower, turretMaxPower);
            prepareStack(2);
            waitForLift();

            //grab Cone 2
            closeGrabber();
            sleep(200);
            setLift(DR4B_LOWJUNCTION);
            waitForLift();
            setV4B(V4B_HORIZONTAL);
            setTurretPosition(autoSouthWest+20);
            waitForTurret(turretMinPower, turretMaxPower);
            setLift(DR4B_LOWJUNCTION - 75);
            waitForLift();
            openGrabber();
            setV4B(V4B_VERTICAL);
            sleep(250);
            setTurretPosition(NORTH);
            waitForTurret(turretMinPower, turretMaxPower);
            prepareStack(3);
           // sleep(200);
            drive.followTrajectorySequence(traj4);
            waitForLift();

            //grab Cone 3
            closeGrabber();
            sleep(200);
            setLift(DR4B_LOWJUNCTION);
            waitForLift();
            setV4B(V4B_HORIZONTAL);
            setTurretPosition(autoSouthWest);
            waitForTurret(turretMinPower, turretMaxPower);
            setLift(DR4B_LOWJUNCTION - 75);
            waitForLift();
            openGrabber();
            setV4B(V4B_VERTICAL);
            sleep(250);
            setTurretPosition(NORTH);
            waitForTurret(turretMinPower, turretMaxPower);
            prepareStack(4);
            //sleep(200);
            drive.followTrajectorySequence(traj5);
            waitForLift();

            //grab Cone 4
            closeGrabber();
            sleep(200);
            setLift(DR4B_LOWJUNCTION);
            waitForLift();
            setV4B(V4B_HORIZONTAL);
            setTurretPosition(autoSouthWest - 10);
            waitForTurret(turretMinPower, turretMaxPower);
            setLift(DR4B_LOWJUNCTION - 75);
            waitForLift();
            openGrabber();
            setV4B(V4B_VERTICAL);
            sleep(250);
            setTurretPosition(NORTH);
            waitForTurret(turretMinPower, turretMaxPower);
            prepareStack(5);
            //sleep(200);
            drive.followTrajectorySequence(traj6);
            waitForLift();

            //grab Cone 5
            closeGrabber();
            sleep(200);
            setLift(DR4B_LOWJUNCTION);
            waitForLift();
            setV4B(V4B_HORIZONTAL);
            setTurretPosition(autoSouthWest - 40);
            waitForTurret(turretMinPower, turretMaxPower);
            setLift(DR4B_LOWJUNCTION - 75);
            waitForLift();
            openGrabber();
            setV4B(V4B_VERTICAL);
            sleep(250);
            closeGrabber();
            setLift(DR4B_GROUNDFLOORTURRETCLEARANCE);
            setTurretPosition(NORTH);
            sleep(50);
            setV4B(V4B_RETRACTED);


            drive.followTrajectorySequence(trajFinal);
            waitForTurret(turretMinPower, turretMaxPower);
        }
    }

    public void setTurretPosition(int position){
        motorTurret.setTargetPosition(position);
        motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(turretDefaultPower);
    }

    public void waitForTurret(double lowConst, double highConst){
        int distance = Math.abs(motorTurret.getCurrentPosition() - motorTurret.getTargetPosition());
        while (Math.abs(motorTurret.getCurrentPosition() - motorTurret.getTargetPosition()) > 3){
            double power = Range.clip((-1 * (Math.pow(Math.abs((Math.abs(motorTurret.getTargetPosition() - motorTurret.getCurrentPosition()) / (distance / 2)) - 1), 14)) + 1), turretMinPower, turretMaxPower);
            motorTurret.setPower(power);
        }
    }

    public void waitForLift(){
        while (Math.abs(motorDR4B1.getCurrentPosition() - motorDR4B1.getTargetPosition()) > 6){

        }
    }

    void openGrabber() {
        servoGrabber.setPosition(grabberOpen);
    }

    void closeGrabber() {
        servoGrabber.setPosition(grabberClose);
    }

    void setLift(int position) {
        motorDR4B1.setTargetPosition(position == 0 ? -5 : position);
        motorDR4B1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDR4B2.setTargetPosition(position == 0 ? -5 : position);
        motorDR4B2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (motorDR4B1.getCurrentPosition() > motorDR4B1.getTargetPosition()) {
            motorDR4B1.setPower(DR4B_LOWPOWER);
            motorDR4B2.setPower(DR4B_LOWPOWER);
        } else {
            motorDR4B1.setPower(1);
            motorDR4B2.setPower(1);
        }
    }

    public void setV4B(double position) {
        servoV4BL.setPosition(position * V4B_SCALELEFT);
        servoV4BR.setPosition(position);
    }

    public void setLow(){
        setLift(DR4B_LOWJUNCTION);
        setV4B(V4B_LOWMID);
    }

    public void setHigh(){
        setLift(DR4B_MIDHIGHJUNCTION);
        setV4B(V4B_HIGHJUNCTION);
    }

    public void setMid(){
        setLift(DR4B_MIDHIGHJUNCTION);
        setV4B(V4B_LOWMID);
    }

    public void prepareStack(int coneNumber){
        setLift(DR4B_GROUNDFLOORTURRETCLEARANCE);
        setV4B(starterStack.get(coneNumber - 1));
        openGrabber();
    }

    public void updateTimeStamp() {
        timeStamp = getRuntime();
    }

    public void pause(double seconds){
        updateTimeStamp();
        while (getRuntime() - timeStamp < 1){

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
