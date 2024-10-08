package com.example.meepmeep7462;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep7462 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-31.00, 62.5, Math.toRadians(-90.00)))
                        .splineToConstantHeading(new Vector2d(-31.0, 30.00), Math.toRadians(-90.0))
                        .splineToConstantHeading(new Vector2d(-35.0, 20.00), Math.toRadians(-130.0))
                        .splineToConstantHeading(new Vector2d(-40.0, 16.0), Math.toRadians(180.0))
                        .splineToConstantHeading(new Vector2d(-45.0, 20.0), Math.toRadians(110.0))
                        .splineToConstantHeading(new Vector2d(-47.0, 57.91), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(-50, 20.0), Math.toRadians(-135.0))
                        .splineToConstantHeading(new Vector2d(-52.0, 19.0), Math.toRadians(180.0))
                        .splineToConstantHeading(new Vector2d(-54.0, 20.0), Math.toRadians(135.0))
                        .splineToConstantHeading(new Vector2d(-57.0, 57.91), Math.toRadians(90.0))
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(-57, 20.0), Math.toRadians(-90.0))
                        .splineToConstantHeading(new Vector2d(-63.0, 20.0), Math.toRadians(90.0))
                        .splineToConstantHeading(new Vector2d(-63.0, 57.91), Math.toRadians(90.0))
                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}