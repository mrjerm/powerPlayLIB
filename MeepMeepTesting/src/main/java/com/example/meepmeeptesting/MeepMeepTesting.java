package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600, 240);

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

        RoadRunnerBotEntity speed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 52.48291908330528, Math.toRadians(273.36816), Math.toRadians(273.36816), 11.58)
                .setDimensions(14.1, 14.1)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(38,-62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(39, -21, Math.toRadians(88)))

                                        //curve to starter stackn
                                .splineToSplineHeading(new Pose2d(45.6,-12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(56,-12, Math.toRadians(0)), Math.toRadians(0))



                .build());

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 52.48291908330528, Math.toRadians(273.36816), Math.toRadians(273.36816), 11.58)
                .setDimensions(14.1, 14.1)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36,62, Math.toRadians(-90)))
                                //preload
                                .lineToLinearHeading(new Pose2d(36,23, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(36,12,Math.toRadians(-90)))

                                //curve to starter stackn
                                .lineToLinearHeading(new Pose2d(58,12,Math.toRadians(0)))

                                //GO TO HIGH
                                .lineToLinearHeading(new Pose2d(22.5,12,Math.toRadians(0)))

                                //GO BACK
                                .lineToLinearHeading(new Pose2d(58,12,Math.toRadians(0)))

                                //GO TO HIGH
                                .lineToLinearHeading(new Pose2d(22.5,12,Math.toRadians(0)))

                                //GO BACK
                                .lineToLinearHeading(new Pose2d(58,12,Math.toRadians(0)))

                                //GO TO HIGH
                                .lineToLinearHeading(new Pose2d(22.5,12,Math.toRadians(0)))

                                //GO BACK
                                .lineToLinearHeading(new Pose2d(58,12,Math.toRadians(0)))

                                //GO TO HIGH
                                .lineToLinearHeading(new Pose2d(22.5,12,Math.toRadians(0)))

                                //GO BACK
                                .lineToLinearHeading(new Pose2d(58,12,Math.toRadians(0)))

                                //GO TO HIGH
                                .lineToLinearHeading(new Pose2d(22.5,12,Math.toRadians(0)))







                                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(speed)
                .start();
    }
}