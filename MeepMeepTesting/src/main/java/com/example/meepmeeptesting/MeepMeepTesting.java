package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import static java.lang.Math.PI;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Vector;


public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 63, Math.toRadians(90)))
                .setTangent(-PI/2)
                .lineToY(36) //fwd to bar
                //hang specimen #1
                .lineToY(50) //pull out from bar a bit
                .splineToConstantHeading(new Vector2d(-40,25), -PI/2) //to blue sample
                .splineToConstantHeading(new Vector2d(-44, 10), -PI/2) //to blue sample
                .splineToConstantHeading(new Vector2d(-44, 48), -PI/2) //to human player zone
                .splineToConstantHeading(new Vector2d(-44, 10), -PI/2) //to blue sample #2
                .splineToConstantHeading(new Vector2d(-54, 12), -PI/2) //to blue sample #2
                .splineToConstantHeading(new Vector2d(-54, 48), -PI/2) //to human player zone
                .lineToY(45) //pull out from wall a bit
                .splineToConstantHeading(new Vector2d(-35, 56), -PI/2) //to wall - to pick up specimen
                //grab specimen #2
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2) //to submersible
                //place specimen #2
                .lineToY(45) //pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-54, 10), -PI/2) //to blue sample #3
                .splineToConstantHeading(new Vector2d(-61, 12), -PI/2)//to blue sample #3
                .splineToConstantHeading(new Vector2d(-61, 48), -PI/2) //to human player zone
                .lineToY(45) // pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-35, 56), -PI/2) // to wall - to pick up specimen
                //grab specimen #3
                .lineToY(45) // pull out from wall a bit
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2) //to submersible
                //place specimen #3
                .lineToY(45) // pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-35, 56), -PI/2) // to wall - to pick up specimen
                //grab specimen #4
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2) //to submersible
                //place specimen #4
                .lineToY(45) // pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-35, 56), -PI/2) // to wall - to pick up specimen
                //grab specimen #5
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2) //to submersible
                //place specimen #5




                /*.strafeTo(new Vector2d(56.0, 57.5))
                .turn(3*PI/4)
                //dump
                .strafeTo(new Vector2d(60.0, 40.5))
                .turn(PI/4)
                //pick up
                .strafeTo(new Vector2d(56.0, 57.5))
                .turn(-PI/4)
                //dump
                .strafeTo(new Vector2d(61.0,55.0))
                .turn(PI*0.6)
                //pick up
                .strafeTo(new Vector2d(56.0, 57.5))
                .turn(-PI*0.6)
                //dump
                .strafeTo(new Vector2d(30.0,0.0))*/
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}