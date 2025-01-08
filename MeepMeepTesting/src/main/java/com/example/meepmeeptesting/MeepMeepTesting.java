package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 63, Math.toRadians(90)))
                .setTangent(-PI/2)
                .lineToY(34) //fwd to bar
                //hang specimen #1
                .lineToY(50) //pull out from bar a bit
                //.splineToConstantHeading(new Vector2d(-40,25), -PI/2) //to blue sample
                .splineToConstantHeading(new Vector2d(-45, 10), PI/2) //to blue sample
                .splineToConstantHeading(new Vector2d(-45, 48), -PI/2) //to human player zone
                //.splineToConstantHeading(new Vector2d(-45, 10), -PI/2) //to blue sample #2
                .splineToConstantHeading(new Vector2d(-55, 10), 3 * PI/4) //to blue sample #2
                .setTangent(-PI/2)
                .splineToConstantHeading(new Vector2d(-55, 48), -PI/2) //to human player zone
                //.lineToY(45) //pull out from wall a bit
                .splineToConstantHeading(new Vector2d(-35, 56), PI/2) //to wall - to pick up specimen
                //grab specimen #2
                .setTangent(-PI/2)
                .splineToConstantHeading(new Vector2d(0, 34), 0) //to submersible
                //place specimen #2
                .setTangent(-PI/2)
                .lineToY(45) //pull out from submersible a bit

                //.splineToConstantHeading(new Vector2d(-54, 10), PI/4) //to blue sample #3
                .splineToConstantHeading(new Vector2d(-61, 12), PI/2)//to blue sample #3
                .splineToConstantHeading(new Vector2d(-61, 48), -PI/2) //to human player zone
                //.lineToY(45) // pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-35, 56), PI/2) // to wall - to pick up specimen
                //grab specimen #3
                //.lineToY(45) // pull out from wall a bit
                .setTangent(-PI/2)
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2) //to submersible ;
                //place specimen #3

                .lineToY(45) // pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-35, 56), 0) // to wall - to pick up specimen
                //grab specimen #4
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2) //to submersible
                //place specimen #4
                .lineToY(45) // pull out from submersible a bit


                .splineToConstantHeading(new Vector2d(-35, 56), 0) // to wall - to pick up specimen
                //grab specimen #5
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2)//to submersible
                //place specimen #5

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}