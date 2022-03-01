package org.frcteam2910.c2020.util;

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

//     private static final String EIGHT_BALL_AUTO_PART_ONE_NAME = "autos/8BallAuto/8BallAutoPart1.path";
//     private static final String EIGHT_BALL_AUTO_PART_TWO_NAME = "autos/8BallAuto/8BallAutoPart2.path";

    private Trajectory sevenFeet;

    private final Trajectory sCurve;
    
    private final Trajectory StartPosition1ToBall1;
    private final Trajectory TerminalFiveBallPart2;
    private final Trajectory StartPosition1ToBall3;
    private final Trajectory LoadToShootPosition;
    private final Trajectory TerminalToLoadPosition;
    private final Trajectory StartPosition1ToShoot;
    private final Trajectory FiveBallEndToShoot;

    private final Trajectory StartPosition0ToBall1;
    private final Trajectory ThreeBallPartTwo;
    private final Trajectory HangarFourBallPartOne;
    private final Trajectory HangarFourBallPartTwo;
    private final Trajectory simpleShootThree;

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
                new SimplePathBuilder(new Vector2(257, -219), Rotation2.fromDegrees(200.8))
                        .lineTo(new Vector2(209, -244), Rotation2.fromDegrees(207.8)) //-244
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
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
                new SimplePathBuilder(new Vector2(300, -254), Rotation2.fromDegrees(270)) //51 -268
                        .lineTo(new Vector2(297, -294), Rotation2.fromDegrees(260)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        ThreeBallPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(297, -294), Rotation2.fromDegrees(260)) //51 -268
                        .lineTo(new Vector2(242, -248), Rotation2.fromDegrees(207.8)) //209 -259
                        .arcTo(new Vector2(216, -245), new Vector2(230, -260))
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

        sevenFeet = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.ZERO)
                        .lineTo(new Vector2(84, 0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        simpleShootThree = new Trajectory(
                new SimplePathBuilder(Vector2.ZERO, Rotation2.ZERO)
                        .lineTo(new Vector2(40.0, 0.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
    }

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

    public Trajectory get_TerminalFiveBallPart2(){
        return TerminalFiveBallPart2;
}

    public Trajectory get_StartPosition1ToBall3(){
            return StartPosition1ToBall3;
    }

    public Trajectory get_TerminalToLoadPosition(){
            return TerminalToLoadPosition;
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

    public Trajectory get_FiveBallEndToShoot(){
            return FiveBallEndToShoot;
    }

    public Trajectory getSimpleShootThree() { return simpleShootThree; }
}
