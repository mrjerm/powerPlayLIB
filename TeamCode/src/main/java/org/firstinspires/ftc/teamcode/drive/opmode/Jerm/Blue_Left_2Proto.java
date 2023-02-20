package org.firstinspires.ftc.teamcode.drive.opmode.Jerm;

import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_GROUNDFLOORTURRETCLEARANCE;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_LOWJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_LOWPOWER;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.DR4B_MIDHIGHJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_HIGHJUNCTION;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_LOWMID;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_RETRACTED;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_SCALELEFT;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.V4B_VERTICAL;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.east;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.grabberClose;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.grabberOpen;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.north;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.starterStack;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.west;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
@Disabled
public class Blue_Left_2Proto extends LinearOpMode {

    static double timeStamp;

    public DcMotorEx motorDR4B;

    public Servo servoTurret;
    public Servo servoGrabber;
    public Servo servoV4BL, servoV4BR;

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

        Pose2d startPose = new Pose2d(38, 62, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        //vision
        //servo initialization

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        motorDR4B = hardwareMap.get(DcMotorEx.class, "Motor DR4B");
        motorDR4B.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorDR4B.setDirection(DcMotorEx.Direction.REVERSE);
        motorDR4B.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        servoTurret = hardwareMap.get(Servo.class, "Servo Turret");
        servoGrabber = hardwareMap.get(Servo.class, "Servo Intake");
        servoGrabber.setPosition(grabberClose);
        servoV4BL = hardwareMap.get(Servo.class, "Servo V4BL");
        servoV4BR = hardwareMap.get(Servo.class, "Servo V4BR");
        servoV4BL.setDirection(Servo.Direction.REVERSE);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    servoTurret.setPosition(east); //prepare turret for dropping preload
                })
                .lineToLinearHeading(new Pose2d(37.5, 22.5, Math.toRadians(-92)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    prepareStack(1); //prepare v4b + dr4b for starter stack cone 1
                })
                .lineToLinearHeading(new Pose2d(38.5, 12, Math.toRadians(-92)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .lineToLinearHeading(new Pose2d(54.5, 8, Math.toRadians(-6)))
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    servoTurret.setPosition(west); //point turret towards (1, 2) junction
                })
                .lineToLinearHeading(new Pose2d(48, 9, Math.toRadians(-6)))
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(54.5, 9, Math.toRadians(-4)))
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    servoTurret.setPosition(west); //point turret towards (1, 2) junction
                })
                .lineToLinearHeading(new Pose2d(48, 9, Math.toRadians(-4)))
                .build();

        servoV4BL.setPosition(V4B_RETRACTED);
        servoV4BR.setPosition(V4B_RETRACTED);
        servoTurret.setPosition(north);

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

                        .lineToLinearHeading(new Pose2d(56, 10, Math.toRadians(-6)))

                        .build();
            } else if (tagOfInterest.id == MIDDLE) {
                trajFinal = drive.trajectorySequenceBuilder(traj6.end())

                        .lineToLinearHeading(new Pose2d(36, 10, Math.toRadians(-6)))
                        .build();
            } else {
                trajFinal = drive.trajectorySequenceBuilder(traj6.end())
                        .lineToLinearHeading(new Pose2d(11, 10, Math.toRadians(-6)))

                        .build();
            }
        }

        if (opModeIsActive()){
            //prepare to drop off preload
            setMid();
            //move to (4, 2) junction
            drive.followTrajectorySequence(traj1);
            //drop dr4b to align cone on junction
            setLift(DR4B_MIDHIGHJUNCTION - 100);
            pause(0.3);
            //score preload cone
            openGrabber();

            //retract v4b to prepare to move to starter stack
            setV4B(V4B_VERTICAL);
            pause(0.3);
            //point turret towards starter stack
            servoTurret.setPosition(north);
            pause(0.2);

            //move to starter stack
            drive.followTrajectorySequence(traj2);
            drive.turn(Math.toRadians(90));
            drive.followTrajectorySequence(traj3);
            //grab cone
            closeGrabber();
            pause(0.3);
            //lift cone from stack to avoid interference
            setLift(DR4B_LOWJUNCTION);
            pause(0.3);

            //move to (5, 2) junction
            drive.followTrajectorySequence(traj4);
            pause(0.1);
            //drop dr4b to align cone on junction
            setLift(DR4B_LOWJUNCTION - 100);
            pause(0.2);
            //score starter stack cone 1
            openGrabber();
            setV4B(V4B_VERTICAL);
            pause(0.3);

            servoTurret.setPosition(north); //point turret towards starter stack
            pause(0.5);
            prepareStack(2); //prepare v4b + dr4b for starter stack cone 2
            pause(0.5);
            //move to starter stack to grab cone 2
            drive.followTrajectorySequence(traj5);
            //grab cone
            closeGrabber();
            pause(0.3);
            //lift cone from stack to avoid interference
            setLift(DR4B_LOWJUNCTION);
            pause(0.3);

            //move to (5, 2) junction
            drive.followTrajectorySequence(traj6);
            pause(0.1);
            //drop dr4b to align cone on junction
            setLift(DR4B_LOWJUNCTION - 100);
            pause(0.2);
            //score starter stack cone 1
            openGrabber();

            setV4B(V4B_RETRACTED);
            pause(0.3);

            //retract everything, prepare for parking and teleop
            closeGrabber();
            setLift(DR4B_GROUNDFLOORTURRETCLEARANCE);
            drive.followTrajectorySequence(trajFinal);
            drive.turn(Math.toRadians(-90));

        }
    }

    void openGrabber() {
        servoGrabber.setPosition(grabberOpen);
    }

    void closeGrabber() {
        servoGrabber.setPosition(grabberClose);
    }

    void setLift(int position) {
        motorDR4B.setTargetPosition(position);
        motorDR4B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (motorDR4B.getCurrentPosition() > motorDR4B.getTargetPosition()) {
            motorDR4B.setPower(DR4B_LOWPOWER);
        } else {
            motorDR4B.setPower(1);
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
