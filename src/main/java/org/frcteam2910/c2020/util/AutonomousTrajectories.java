package org.frcteam2910.c2020.util;

import org.frcteam2910.c2020.commands.auton.HangarFourBallSteal;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.io.PathReader;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;
    private final Trajectory HangarTwoBallStealOne;
    private final Trajectory HangarTwoBallStealOnePlace;
    private final Trajectory HangarTwoBallStealOnePlaceBackup;
    private final Trajectory HangarTwoBallStealTwo;
    private final Trajectory HangarTwoBallStealTwoPlace;

//     private static final String EIGHT_BALL_AUTO_PART_ONE_NAME = "autos/8BallAuto/8BallAutoPart1.path";
//     private static final String EIGHT_BALL_AUTO_PART_TWO_NAME = "autos/8BallAuto/8BallAutoPart2.path";

    private Trajectory sevenFeet;

    private final Trajectory sCurve;
    
    private final Trajectory StartPosition1ToBall1;
    private final Trajectory TerminalFiveBallPart2;
    private final Trajectory StartPosition1ToBall3;
    private final Trajectory LoadToShootPosition;
    private final Trajectory TerminalToLoadPosition;
    private final Trajectory TerminalToSingleBallLoad;
    private final Trajectory SingleBallLoadToShootPosition;
    private final Trajectory StartPosition1ToShoot;
    private final Trajectory FiveBallEndToShoot;
    private final Trajectory MidPositionFourBallPart2;

    private final Trajectory StartPosition0ToBall1;
    private final Trajectory ThreeBallPartTwo;
    private final Trajectory HangarFourBallPartOne;
    private final Trajectory HangarFourBallPartTwo;
    private final Trajectory HangarFourBallStealPartOne;
    private final Trajectory simpleShootThree;
    private final Trajectory TerminalStealPartOne;
    private final Trajectory TerminalStealPartTwo;
    private final Trajectory TerminalBallStealPlace;
    private final Trajectory TerminalStealOnePlace;
    private final Trajectory HangarStealTwoDirect;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);
        
        TrajectoryConstraint[] mediumConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        mediumConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(9.0 * 12.0);
        mediumConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(8.0 * 12.0);

        sCurve = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(60, 0), Rotation2.ZERO)
                        .arcTo(new Vector2(100, 40), new Vector2(60, 40))
                        .lineTo(new Vector2(100, 60), Rotation2.ZERO)
                        .arcTo(new Vector2(140, 100), new Vector2(140, 60), Rotation2.ZERO)
                        .lineTo(new Vector2(200, 100))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        StartPosition1ToBall1 = new Trajectory(
                new SimplePathBuilder(new Vector2(260, -240), Rotation2.fromDegrees(200.8))
                        .lineTo(new Vector2(200, -260), Rotation2.fromDegrees(207.8)) //-244
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        MidPositionFourBallPart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(200, -260), Rotation2.fromDegrees(207.8))
                        .lineTo(new Vector2(54, -280), Rotation2.fromDegrees(225))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        TerminalFiveBallPart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(200, -260), Rotation2.fromDegrees(207.8))
                        .lineTo(new Vector2(54, -280), Rotation2.fromDegrees(225))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        StartPosition1ToBall3 = new Trajectory(
                new SimplePathBuilder(new Vector2(60, -260), Rotation2.fromDegrees(223.8)) //51 -268
                        .lineTo(new Vector2(51, -268), Rotation2.fromDegrees(223.8)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        TerminalToLoadPosition = new Trajectory(
                new SimplePathBuilder(new Vector2(54, -280), Rotation2.fromDegrees(225.0)) //51 -268
                        .lineTo(new Vector2(65, -265), Rotation2.fromDegrees(225.0)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        LoadToShootPosition = new Trajectory(
                new SimplePathBuilder(new Vector2(65, -265), Rotation2.fromDegrees(225.0)) //51 -268
                        .lineTo(new Vector2(160, -200), Rotation2.fromDegrees(188.0)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        TerminalToSingleBallLoad = new Trajectory(
                new SimplePathBuilder(new Vector2(54, -280), Rotation2.fromDegrees(225.0)) //51 -268
                        .lineTo(new Vector2(35, -235), Rotation2.fromDegrees(225.0)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        SingleBallLoadToShootPosition = new Trajectory(
                new SimplePathBuilder(new Vector2(35, -235), Rotation2.fromDegrees(225.0)) //51 -268
                        .lineTo(new Vector2(130, -200), Rotation2.fromDegrees(188.0)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        StartPosition1ToShoot = new Trajectory(
                new SimplePathBuilder(new Vector2(257, -219), Rotation2.fromDegrees(223.8)) //51 -268
                        .lineTo(new Vector2(209, -259), Rotation2.fromDegrees(223.8)) //209 -259
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        FiveBallEndToShoot = new Trajectory(
                new SimplePathBuilder(new Vector2(257, -219), Rotation2.fromDegrees(223.8)) //51 -268
                        .lineTo(new Vector2(126, -241), Rotation2.fromDegrees(223.8)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        StartPosition0ToBall1 = new Trajectory(
                new SimplePathBuilder(new Vector2(295, -254), Rotation2.fromDegrees(270)) //51 -268
                        .lineTo(new Vector2(297, -294), Rotation2.fromDegrees(255)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        ThreeBallPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(297, -294), Rotation2.fromDegrees(255)) //51 -268
                        .lineTo(new Vector2(252, -235), Rotation2.fromDegrees(207.8)) //209 -259
                        .arcTo(new Vector2(220, -232), new Vector2(236, -250))
                        .lineTo(new Vector2(200, -260), Rotation2.fromDegrees(207.8))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        HangarFourBallPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(237, -120), Rotation2.fromDegrees(135)) //51 -268
                        .lineTo(new Vector2(206, -89), Rotation2.fromDegrees(148)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        HangarFourBallPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(206, -89), Rotation2.fromDegrees(147)) //51 -268
                        .lineTo(new Vector2(54, -280), Rotation2.fromDegrees(225.0)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        HangarTwoBallStealOne = new Trajectory(
                new SimplePathBuilder(new Vector2(206, -89), Rotation2.fromDegrees(148)) //51 -268
                        .lineTo(new Vector2(235, -80), Rotation2.fromDegrees(90)) //209 -259
                        .lineTo(new Vector2(235, -50), Rotation2.fromDegrees(90))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        HangarTwoBallStealOnePlace = new Trajectory(
                new SimplePathBuilder(new Vector2(235, -50), Rotation2.fromDegrees(90)) //51 -268
                        .lineTo(new Vector2(250,-130), Rotation2.fromDegrees(335)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        HangarStealTwoDirect = new Trajectory(
                new SimplePathBuilder(new Vector2(235, -50), Rotation2.fromDegrees(90)) //51 -268
                .lineTo(new Vector2(210,-160), Rotation2.fromDegrees(215)) //209 -259
                .lineTo(new Vector2(180,-180), Rotation2.fromDegrees(210)) //209 -259
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        HangarTwoBallStealTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(250, -130), Rotation2.fromDegrees(335)) //51 -268
                        .lineTo(new Vector2(210,-160), Rotation2.fromDegrees(215)) //209 -259
                        .lineTo(new Vector2(180,-180), Rotation2.fromDegrees(210)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        HangarTwoBallStealTwoPlace = new Trajectory(
                new SimplePathBuilder(new Vector2(188,-184), Rotation2.fromDegrees(215)) //51 -268
                        .lineTo(new Vector2(250,-130), Rotation2.fromDegrees(335))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        HangarTwoBallStealOnePlaceBackup = new Trajectory(
                new SimplePathBuilder(new Vector2(250, -130), Rotation2.fromDegrees(335)) //51 -268
                        .lineTo(new Vector2(230,-140), Rotation2.fromDegrees(335)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );



        HangarFourBallStealPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(130, -200), Rotation2.fromDegrees(225.0))
                        .lineTo(new Vector2(140, -200), Rotation2.fromDegrees(20.0))
                        .lineTo(new Vector2(160, -200), Rotation2.fromDegrees(20.0))
                        .lineTo(new Vector2(265, -155), Rotation2.fromDegrees(-20.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        TerminalStealPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(297, -294), Rotation2.fromDegrees(255))
                        .lineTo(new Vector2(349, -292), Rotation2.fromDegrees(0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        TerminalStealPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(349, -292), Rotation2.fromDegrees(280))
                        .lineTo(new Vector2(185, -207), Rotation2.fromDegrees(135))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        TerminalBallStealPlace = new Trajectory(
                new SimplePathBuilder(new Vector2(185,-207), Rotation2.fromDegrees(135)) //51 -268
                        .lineTo(new Vector2(260, -140), Rotation2.fromDegrees(335)) //209 -259
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );


        sevenFeet = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.ZERO)
                        .lineTo(new Vector2(84, 0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        TerminalStealOnePlace = new Trajectory(
                new SimplePathBuilder(new Vector2(349, -292), Rotation2.fromDegrees(280))
                        .lineTo(new Vector2(224, -208), Rotation2.fromDegrees(48))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        simpleShootThree = new Trajectory(
                new SimplePathBuilder(Vector2.ZERO, Rotation2.ZERO)
                        .lineTo(new Vector2(40.0, 0.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
    }

    @SuppressWarnings("unused")
    private Path getPath(String name) throws IOException {
        InputStream in = getClass().getClassLoader().getResourceAsStream(name);
        if (in == null) {
            throw new FileNotFoundException("Path file not found: " + name);
        }

        try (PathReader reader = new PathReader(new InputStreamReader(in))) {
            return reader.read();
        }
    }

    public Trajectory getSevenFeet(){
        return sevenFeet;
    }

    public Trajectory get_sCurve(){
            return sCurve;
    }

    public Trajectory get_StartPosition1ToBall1(){
        return StartPosition1ToBall1;
}

    public Trajectory get_MidPositionFourBallPart2(){
        return MidPositionFourBallPart2;
    }

    public Trajectory get_TerminalFiveBallPart2(){
        return TerminalFiveBallPart2;
}

    public Trajectory get_StartPosition1ToBall3(){
            return StartPosition1ToBall3;
    }

    public Trajectory get_TerminalToLoadPosition(){
            return TerminalToLoadPosition;
    }

    public Trajectory get_TerminalToSingleBallLoadPosition(){
        return TerminalToSingleBallLoad;
    }
    public Trajectory get_SingleBallLoadToShootPosition(){
        return SingleBallLoadToShootPosition;
    }

    public Trajectory get_LoadToShootPosition(){
        return LoadToShootPosition;
    }

    public Trajectory get_StartPosition1ToShoot(){
            return StartPosition1ToShoot;
    }

    public Trajectory get_StartPosition0ToBall1(){
            return StartPosition0ToBall1;
    }

    public Trajectory get_ThreeBallPartTwo(){
            return ThreeBallPartTwo;
    }

    public Trajectory get_HangarFourBallPartOne(){
            return HangarFourBallPartOne;
    }

    public Trajectory get_HangarFourBallPartTwo(){
            return HangarFourBallPartTwo;
    }

    public Trajectory get_HangarFourBallStealPartOne(){
        return HangarFourBallStealPartOne;
    }

    public Trajectory get_FiveBallEndToShoot(){
            return FiveBallEndToShoot;
    }

    public Trajectory getSimpleShootThree() { return simpleShootThree; }

    public Trajectory getHangarTwoBallStealOne(){return HangarTwoBallStealOne;}

    public Trajectory getHangarTwoBallStealOnePlace() {return HangarTwoBallStealOnePlace;}

    public Trajectory getHangarTwoBallStealOnePlaceBackup() {return HangarTwoBallStealOnePlaceBackup;}

    public Trajectory getHangarTwoBallStealTwo(){return HangarTwoBallStealTwo;}

    public Trajectory getHangarTwoBallStealTwoPlace() {return HangarTwoBallStealTwoPlace;}

    public Trajectory getTerminalStealPartOne() {return TerminalStealPartOne;}

    public Trajectory getTerminalStealPartTwo() {return TerminalStealPartTwo;}

    public Trajectory getTerminalStealPlace() {return TerminalBallStealPlace;}

    public Trajectory getTerminalStealOnePlace() {return TerminalStealOnePlace;}

    public Trajectory getHangarStealTwoDirect() {return HangarStealTwoDirect;}
}
