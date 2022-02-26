package org.frcteam2910.c2020.util;

import org.frcteam2910.c2020.Robot;
import org.frcteam2910.c2020.RobotContainer;
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
    private static final String TEN_BALL_AUTO_PART_ONE_NAME = "autos/10BallAuto/10BallAutoPart1.path";
    private static final String TEN_BALL_AUTO_PART_TWO_NAME = "autos/10BallAuto/10BallAutoPart2.path";

    private static final Path EIGHT_BALL_COMPATIBLE_PART_ONE = new SimplePathBuilder(new Vector2(511.75, -148.0), Rotation2.ZERO)
            .lineTo(new Vector2(475.75, -134.25), Rotation2.fromDegrees(18.83))
            .build();
    private static final Path EIGHT_BALL_COMPATIBLE_PART_TWO = new SimplePathBuilder(new Vector2(475.75, -134.25), Rotation2.ZERO)
            .lineTo(new Vector2(324.0, -134.25), Rotation2.ZERO)
            .build();
    private static final Path EIGHT_BALL_COMPATIBLE_PART_THREE = new SimplePathBuilder(new Vector2(324.0, -134.25), Rotation2.ZERO)
            .lineTo(new Vector2(474.0, -114.25), Rotation2.fromDegrees(14.0))
            .build();
    private static final Path EIGHT_BALL_COMPATIBLE_PART_FOUR = new SimplePathBuilder(new Vector2(474.0, -114.25), Rotation2.fromDegrees(14.0))
            .lineTo(new Vector2(324.0,-134.25), Rotation2.fromDegrees(0.0))
            .build();

    private Trajectory eightBallAutoPartOne;
    private Trajectory eightBallAutoPartTwo;
    private Trajectory eightBallAutoPartThree;
    private Trajectory eightBallAutoPartFour;
    private Trajectory tenBallAutoPartOne;
    private Trajectory tenBallAutoPartTwo;
    private Trajectory circuitTenBallAutoPartOne;
    private Trajectory circuitTenBallAutoPartTwo;
    private Trajectory simplesquare;
    private Trajectory square;
    private Trajectory sevenFeet;

    private final Trajectory eightBallCompatiblePartOne;
    private final Trajectory eightBallCompatiblePartTwo;
    private final Trajectory eightBallCompatiblePartThree;
    private final Trajectory eightBallCompatiblePartFour;
    private final Trajectory sCurve;
    
    private final Trajectory StartPosition1ToBall1;
    private final Trajectory StartPosition1ToBall2;
    private final Trajectory StartPosition1ToBall3;
    private final Trajectory StartPosition1ToBall4;
    private final Trajectory StartPosition1ToShoot;

    private final Trajectory StartPosition0ToBall1;
    private final Trajectory ThreeBallPartTwo;

    private final Trajectory simpleShootThree;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);


        eightBallAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.ZERO)
                        .lineTo(new Vector2(200, 0), Rotation2.fromDegrees(90.0))
                        .arcTo(new Vector2(250, 50), new Vector2(200, 50))
                        .arcTo(new Vector2(300, 100), new Vector2(300, 50), Rotation2.fromDegrees(0.0))
                        .lineTo(new Vector2(500, 100))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        simplesquare = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.ZERO)
                        .lineTo(new Vector2(100,0))
                        .lineTo(new Vector2(100,100))
                        .lineTo(new Vector2(0,100))
                        .lineTo(new Vector2(0,0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        sCurve = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(60, 0), Rotation2.ZERO)
                        .arcTo(new Vector2(100, 40), new Vector2(60, 40))
                        .lineTo(new Vector2(100, 60), Rotation2.ZERO)
                        .arcTo(new Vector2(140, 100), new Vector2(140, 60), Rotation2.ZERO)
                        .lineTo(new Vector2(200, 100))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        square = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.fromDegrees(90))
                        .lineTo(new Vector2(50,0), Rotation2.fromDegrees(45))
                        .lineTo(new Vector2(50,100), Rotation2.fromDegrees(135))
                        .lineTo(new Vector2(-50,100), Rotation2.fromDegrees(225))
                        .lineTo(new Vector2(-50,0), Rotation2.fromDegrees(315))
                        .lineTo(new Vector2(0,0), Rotation2.fromDegrees(90))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );//207.8

        StartPosition1ToBall1 = new Trajectory(
                new SimplePathBuilder(new Vector2(257, -219), Rotation2.fromDegrees(200.8))
                        .lineTo(new Vector2(209, -244), Rotation2.fromDegrees(207.8)) //-244
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        StartPosition1ToBall2 = new Trajectory(
                new SimplePathBuilder(new Vector2(209, -244), Rotation2.fromDegrees(200.8))
                        .lineTo(new Vector2(60, -260), Rotation2.fromDegrees(223.8))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        StartPosition1ToBall3 = new Trajectory(
                new SimplePathBuilder(new Vector2(60, -260), Rotation2.fromDegrees(223.8)) //51 -268
                        .lineTo(new Vector2(51, -268), Rotation2.fromDegrees(223.8)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        StartPosition1ToBall4 = new Trajectory(
                new SimplePathBuilder(new Vector2(51, -268), Rotation2.fromDegrees(223.8)) //51 -268
                        .lineTo(new Vector2(60, -260), Rotation2.fromDegrees(223.8)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        StartPosition1ToShoot = new Trajectory(
                new SimplePathBuilder(new Vector2(257, -219), Rotation2.fromDegrees(223.8)) //51 -268
                        .lineTo(new Vector2(209, -259), Rotation2.fromDegrees(223.8)) //209 -259
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        StartPosition0ToBall1 = new Trajectory(
                new SimplePathBuilder(new Vector2(300, -254), Rotation2.fromDegrees(270)) //51 -268
                        .lineTo(new Vector2(297, -294), Rotation2.fromDegrees(270)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        ThreeBallPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(297, -294), Rotation2.fromDegrees(270)) //51 -268
                        .lineTo(new Vector2(221.4, -238.4), Rotation2.fromDegrees(207.8)) //209 -259
                        .lineTo(new Vector2(209, -244), Rotation2.fromDegrees(207.8)) //209 -259
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        eightBallAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(468.0, -67.34), Rotation2.ZERO)
                        .lineTo(new Vector2(459.23, -111.87))
                        .arcTo(new Vector2(432.0, -134.25), new Vector2(432.0, -106.5))
                        .lineTo(new Vector2(324.0, -134.25), Rotation2.fromDegrees(0.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        eightBallAutoPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(324.0, -134.25), Rotation2.fromDegrees(0.0))
                        .arcTo(new Vector2(468.0, -67.34), new Vector2(324.0, 54.16))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        eightBallAutoPartFour = new Trajectory(
                new SimplePathBuilder(new Vector2(468.0, -67.34), Rotation2.fromDegrees(0.0))
                        .arcTo(new Vector2(324, -134.25), new Vector2(324.0, 54.16))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        sevenFeet = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.ZERO)
                        .lineTo(new Vector2(84, 0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        tenBallAutoPartOne = new Trajectory(getPath(TEN_BALL_AUTO_PART_ONE_NAME), trajectoryConstraints, SAMPLE_DISTANCE);
        tenBallAutoPartTwo = new Trajectory(getPath(TEN_BALL_AUTO_PART_TWO_NAME), trajectoryConstraints, SAMPLE_DISTANCE);

        circuitTenBallAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(509.0, -162.0), Rotation2.ZERO)
                        .lineTo(new Vector2(385.51, -99.31), Rotation2.fromDegrees(290.0))
                        .arcTo(new Vector2(385.55, -77.48), new Vector2(390.51, -88.41))
                        .lineTo(new Vector2(418.33, -62.60))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        circuitTenBallAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(418.33, -62.60), Rotation2.fromDegrees(290.0))
                        .lineTo(new Vector2(413.52, -66.18), Rotation2.fromDegrees(290.0))
                        .arcTo(new Vector2(435.87, -94.38), new Vector2(424.28, -80.61))
                        .lineTo(new Vector2(468.0, -67.34), Rotation2.ZERO)
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        eightBallCompatiblePartOne = new Trajectory(EIGHT_BALL_COMPATIBLE_PART_ONE, trajectoryConstraints, SAMPLE_DISTANCE);
        eightBallCompatiblePartTwo = new Trajectory(EIGHT_BALL_COMPATIBLE_PART_TWO, slowConstraints, SAMPLE_DISTANCE);
        eightBallCompatiblePartThree = new Trajectory(EIGHT_BALL_COMPATIBLE_PART_THREE, trajectoryConstraints, SAMPLE_DISTANCE);
        eightBallCompatiblePartFour = new Trajectory(EIGHT_BALL_COMPATIBLE_PART_FOUR, trajectoryConstraints, SAMPLE_DISTANCE);

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

    public Trajectory getEightBallAutoPartOne() {
        return eightBallAutoPartOne;
    }

    public Trajectory getSimpleSquare() {
        return simplesquare;
    }

    public Trajectory getSquare() {
        return square;
    }

    public Trajectory getSevenFeet(){
        return sevenFeet;
    }

    public Trajectory getEightBallAutoPartTwo() {
        return eightBallAutoPartTwo;
    }

    public Trajectory getEightBallAutoPartThree() {
        return eightBallAutoPartThree;
    }

    public Trajectory getEightBallAutoPartFour() {
        return eightBallAutoPartFour;
    }

    public Trajectory getTenBallAutoPartOne() {
        return tenBallAutoPartOne;
    }

    public Trajectory getTenBallAutoPartTwo() {
        return tenBallAutoPartTwo;
    }

    public Trajectory get_sCurve(){
            return sCurve;
    }

    public Trajectory get_StartPosition1ToBall1(){
        return StartPosition1ToBall1;
}

    public Trajectory get_StartPosition1ToBall2(){
        return StartPosition1ToBall2;
}
public Trajectory get_StartPosition1ToBall3(){
        return StartPosition1ToBall3;
}
public Trajectory get_StartPosition1ToBall4(){
        return StartPosition1ToBall4;
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
    public Trajectory getCircuitTenBallAutoPartOne() {
        return circuitTenBallAutoPartOne;
    }

    public Trajectory getCircuitTenBallAutoPartTwo() {
        return circuitTenBallAutoPartTwo;
    }

    public Trajectory getEightBallCompatiblePartOne() {
        return eightBallCompatiblePartOne;
    }

    public Trajectory getEightBallCompatiblePartTwo() {
        return eightBallCompatiblePartTwo;
    }

    public Trajectory getEightBallCompatiblePartThree() {
        return eightBallCompatiblePartThree;
    }

    public Trajectory getEightBallCompatiblePartFour() {
        return eightBallCompatiblePartFour;
    }

    public Trajectory getSimpleShootThree() { return simpleShootThree; }
}
