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
                        drive.trajectorySequenceBuilder(new Pose2d(40.5, 64.5, Math.toRadians(-90))) //start
                                .lineToLinearHeading(new Pose2d(12,64.5, Math.toRadians(-90))) //dropoff dodge
                                .lineToLinearHeading(new Pose2d(12,12, Math.toRadians(-90)))  //dropoff tile
                                .lineToLinearHeading(new Pose2d(-16,8, Math.toRadians(-135)))  // dropoff
                                .waitSeconds(0.5)
                                //.lineToLinearHeading(new Pose2d(-12,12, Math.toRadians(180)))  //turn to cones
                                .lineToLinearHeading(new Pose2d(-12,12, Math.toRadians(180)))  // park 1
                                .build()



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}