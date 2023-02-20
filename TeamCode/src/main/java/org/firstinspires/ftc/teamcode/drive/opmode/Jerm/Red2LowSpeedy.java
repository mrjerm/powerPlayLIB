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
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.south1;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.south2;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.starterStack;
import static org.firstinspires.ftc.teamcode.drive.ConstantsPP.west;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
import org.jetbrains.annotations.NotNull;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Disabled

public class Red2LowSpeedy extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(35.6,-62, Math.toRadians(90));

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

        TrajectoryVelocityConstraint slowVel = new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 5;
            }
        };

        TrajectoryAccelerationConstraint defaultAccel = new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 52.48291908330528;
            }
        };

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(35.6,-62, Math.toRadians(90))).setTangent(Math.toRadians(90))
                //preload
                .splineToSplineHeading(new Pose2d(36,-42.4, Math.toRadians(92)), Math.toRadians(92))

                //curve to starter stack
                .splineToSplineHeading(new Pose2d(36,-25.6, Math.toRadians(86)), Math.toRadians(86))
                .splineToSplineHeading(new Pose2d(38.8,-16.8, Math.toRadians(30)), Math.toRadians(30))
                .splineToSplineHeading(new Pose2d(45.6,-12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(58,-12.8, Math.toRadians(0)), Math.toRadians(0), slowVel, defaultAccel)

                //score 1st of stack on high junction
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(-180), slowVel, defaultAccel)
                .splineToSplineHeading(new Pose2d(42,-12.8, Math.toRadians(0)), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(34,-12.8, Math.toRadians(0)), Math.toRadians(177))
                .splineToSplineHeading(new Pose2d(23.6,-12.8, Math.toRadians(0)), Math.toRadians(-180))

                //back to starter stack
                .splineToSplineHeading(new Pose2d(30.8,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(40,-12.8, Math.toRadians(2)), Math.toRadians(2))
                .splineToSplineHeading(new Pose2d(51.6,-12.8, Math.toRadians(-2)), Math.toRadians(-2))
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(58,-12.8, Math.toRadians(0)), Math.toRadians(0), slowVel, defaultAccel)

                //2nd of stack
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(-180), slowVel, defaultAccel)
                .splineToSplineHeading(new Pose2d(42,-12.8, Math.toRadians(0)), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(34,-12.8, Math.toRadians(0)), Math.toRadians(177))
                .splineToSplineHeading(new Pose2d(23.6,-12.8, Math.toRadians(0)), Math.toRadians(-180))

                //back to starter stack
                .splineToSplineHeading(new Pose2d(30.8,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(40,-12.8, Math.toRadians(2)), Math.toRadians(2))
                .splineToSplineHeading(new Pose2d(51.6,-12.8, Math.toRadians(-2)), Math.toRadians(-2))
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(58,-12.8, Math.toRadians(0)), Math.toRadians(0), slowVel, defaultAccel)

                //3rd of stack
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(-180), slowVel, defaultAccel)
                .splineToSplineHeading(new Pose2d(42,-12.8, Math.toRadians(0)), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(34,-12.8, Math.toRadians(0)), Math.toRadians(177))
                .splineToSplineHeading(new Pose2d(23.6,-12.8, Math.toRadians(0)), Math.toRadians(-180))

                //back to starter stack
                .splineToSplineHeading(new Pose2d(30.8,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(40,-12.8, Math.toRadians(2)), Math.toRadians(2))
                .splineToSplineHeading(new Pose2d(51.6,-12.8, Math.toRadians(-2)), Math.toRadians(-2))
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(58,-12.8, Math.toRadians(0)), Math.toRadians(0), slowVel, defaultAccel)

                //4th of stack
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(-180), slowVel, defaultAccel)
                .splineToSplineHeading(new Pose2d(42,-12.8, Math.toRadians(0)), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(34,-12.8, Math.toRadians(0)), Math.toRadians(177))
                .splineToSplineHeading(new Pose2d(23.6,-12.8, Math.toRadians(0)), Math.toRadians(-180))

                //back to starter stack
                .splineToSplineHeading(new Pose2d(30.8,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(40,-12.8, Math.toRadians(2)), Math.toRadians(2))
                .splineToSplineHeading(new Pose2d(51.6,-12.8, Math.toRadians(-2)), Math.toRadians(-2))
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(58,-12.8, Math.toRadians(0)), Math.toRadians(0), slowVel, defaultAccel)

                //5th of stack
                .splineToSplineHeading(new Pose2d(56,-12.8, Math.toRadians(0)), Math.toRadians(-180), slowVel, defaultAccel)
                .splineToSplineHeading(new Pose2d(42,-12.8, Math.toRadians(0)), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(34,-12.8, Math.toRadians(0)), Math.toRadians(177))
                .splineToSplineHeading(new Pose2d(23.6,-12.8, Math.toRadians(0)), Math.toRadians(-180))
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    servoTurret.setPosition(west); //prepare turret for dropping preload
                })
                .lineToLinearHeading(new Pose2d(38.5, -21, Math.toRadians(88)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    prepareStack(1); //prepare v4b + dr4b for starter stack cone 1
                })
                .lineToLinearHeading(new Pose2d(38.5, -12, Math.toRadians(88)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(60, -14, Math.toRadians(87)))
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    servoTurret.setPosition(south2); //point turret towards (5, 2) junction
                    setLow();
                })
                .lineToLinearHeading(new Pose2d(51.5, -14, Math.toRadians(86)))
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    prepareStack(2); //prepare v4b + dr4b for starter stack cone 2
                })
                .lineToLinearHeading(new Pose2d(60, -14, Math.toRadians(84)))
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    servoTurret.setPosition(south2); //point turret towards (5, 2) junction
                    setLow();
                })
                .lineToLinearHeading(new Pose2d(51.5, -14, Math.toRadians(86)))
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
                trajFinal = drive.trajectorySequenceBuilder(traj4.end())
                        .lineToLinearHeading(new Pose2d(15, -11, Math.toRadians(86)))
                        .build();
            } else if (tagOfInterest.id == MIDDLE) {
                trajFinal = drive.trajectorySequenceBuilder(traj4.end())
                        .lineToLinearHeading(new Pose2d(38, -11, Math.toRadians(86)))
                        .build();
            } else {
                trajFinal = drive.trajectorySequenceBuilder(traj4.end())
                        .lineToLinearHeading(new Pose2d(62, -11, Math.toRadians(86)))
                        .build();
            }
        }

        if (opModeIsActive()){
            drive.followTrajectorySequence(traj);
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
