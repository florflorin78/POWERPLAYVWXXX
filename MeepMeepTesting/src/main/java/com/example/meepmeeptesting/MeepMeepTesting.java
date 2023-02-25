package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.invoke.VolatileCallSite;

public class MeepMeepTesting {

    public  static double START_HEADING = 90.0;
    public  static double START_X1 = 23.5;
    public  static double START_Y1 = -72;
    public  static double START_X2 = -23.5;
    public  static double START_Y2 = -72;
    public  static double PRELOAD_X = -31.6;
    public  static double PRELOAD_Y = -31.3 ;
    public  static double PRELOAD_HEADING = 0.0;
    public  static double PRELOAD_TANGENT = 75.0;
    public  static double TILE_X = -36.0;
    public  static double TILE_Y = -18.5;
    public  static double CONE_STACK_X = -56.6;
    public  static double CONE_STACK_Y = -21.0;
    public  static double MID_FORWARD = 2.85;
    public  static double CONE_STACK_HEADING = -180.0;
    public  static double MID_X = -24.5;
    public  static double MID_Y = -18.8;
    public  static double MID_HEADING = 270.0;
    public  static double PARK_1_X = -57.8;
    public  static double PARK_1_Y = -18.5;
    public  static double PARK_2_X = -36.0;
    public  static double PARK_2_Y = -18.0;
    public  static double PARK_3_X = -11.5;
    public  static double PARK_3_Y = -18.0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 45, Math.toRadians(226.25466078530798), Math.toRadians(184.02607784577722), 13.97)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24, -4, Math.toRadians(110)))


//                                  .splineToSplineHeading(new Pose2d(38, -45, Math.toRadians(90)), Math.toRadians(70))
//                                  .splineToSplineHeading(new Pose2d(24, -4, Math.toRadians(110)), Math.toRadians(-200))
                                .lineToLinearHeading (new Pose2d(35,-10,Math.toRadians(0)))



//                                .strafeLeft(25)
//                                .splineToLinearHeading(new Pose2d(15, 0, Math.toRadians(PRELOAD_HEADING)), Math.toRadians(70))
//                                .strafeRight(2)
//                                .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
//                                .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
//                                .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
//                                .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
//                                .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
//                                .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
//                                .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))

                                //caz 1 11.5 -11.5
//                                .lineToSplineHeading(new Pose2d(11.5, -11.5, Math.toRadians(90)))
                                //caz 2 35.5 -11.5
//                                .lineToSplineHeading(new Pose2d(35.5, -11.5, Math.toRadians(90)))
                                //caz 3 58.5 -11.5
//                                .lineToSplineHeading(new Pose2d(58.5, -11.5, Math.toRadians(90)))
                                .build()


                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}