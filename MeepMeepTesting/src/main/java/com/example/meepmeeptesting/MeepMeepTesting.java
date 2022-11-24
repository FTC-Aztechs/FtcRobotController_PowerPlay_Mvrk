package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(39.5, 64.5, Math.toRadians(-90)))
                //preload
                .lineToLinearHeading(new Pose2d(12,64.5, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(12,14, Math.toRadians(-90)))
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(19,4, Math.toRadians(-50)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(12,14, Math.toRadians(-90)))
                //pickup cone 5

                .lineToLinearHeading(new Pose2d(65,12, Math.toRadians(0)))

                .waitSeconds(1.5)

                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(27.5,4, Math.toRadians(-130)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))
                //cone 4

                .lineToLinearHeading(new Pose2d(65,12, Math.toRadians(0)))

                .waitSeconds(1.5)

                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(27.5,4, Math.toRadians(-130)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))
                //cone 3

                .lineToLinearHeading(new Pose2d(65,12, Math.toRadians(0)))

                .waitSeconds(1.5)

                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(27.5,4, Math.toRadians(-130)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))
                //cone 2

                .lineToLinearHeading(new Pose2d(65,12, Math.toRadians(0)))

                .waitSeconds(1.5)

                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(27.5,4, Math.toRadians(-130)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))
                //cone 1

                .lineToLinearHeading(new Pose2d(65,12, Math.toRadians(0)))

                .waitSeconds(1.5)

                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(27.5,4, Math.toRadians(-130)))

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(36,12, Math.toRadians(-90)))
                //park

                .lineToLinearHeading(new Pose2d(60,12, Math.toRadians(0)))
                .build()



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}