package org.frcteam2910.c2020.util;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.auton.TerminalFiveBall;
import org.frcteam2910.c2020.commands.auton.MidPositionFourBall;
import org.frcteam2910.c2020.commands.auton.TerminalThreeBall;
import org.frcteam2910.c2020.commands.auton.*;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Terminal Five Ball", AutonomousMode.TerminalFiveBall);
        autonomousModeChooser.addOption("7 Feet", AutonomousMode.SEVEN_FEET);
        autonomousModeChooser.addOption("sCurve", AutonomousMode.S_CURVE);
        autonomousModeChooser.addOption("Mid Position Four Ball", AutonomousMode.MidPositionFourBall);
        autonomousModeChooser.addOption("Terminal Three Ball", AutonomousMode.TerminalThreeBall);
        autonomousModeChooser.addOption("Terminal Two Ball", AutonomousMode.TerminalTwoBall);
        autonomousModeChooser.addOption("Hangar Four Ball", AutonomousMode.HangarFourBall);
        autonomousModeChooser.addOption("Hangar Two Ball", AutonomousMode.HANGAR_TWO_BALL);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    private SequentialCommandGroup get10BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenBallAutoPartOne());
        //command.addCommands(new FollowTrajectoryCommand(drivetrainSubsystem, trajectories.getTenBallAutoPartTwo()));
        //command.addCommands(new TargetWithShooterCommand(shooterSubsystem, visionSubsystem, xboxController));

        return command;
    }

    private Command get8BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        //reset robot pose
        resetRobotPose(command, container, trajectories.getEightBallAutoPartOne());
        //follow first trajectory and shoot
        follow(command, container, trajectories.getEightBallAutoPartOne());
        //follow second trajectory and shoot
 
        // follow(command, container, trajectories.getEightBallAutoPartThree());
        // follow(command, container, trajectories.getEightBallAutoPartFour());

        return command;
    }

    private Command getSevenFeet(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getSevenFeet());

        follow(command, container, trajectories.getSevenFeet());

        return command;
    }

    private Command getSimpleSquareCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        //reset robot pose
        resetRobotPose(command, container, trajectories.getSimpleSquare());
        //follow first trajectory and shoot
        //follow second trajectory and shoot
 
        // follow(command, container, trajectories.getEightBallAutoPartThree());
        // follow(command, container, trajectories.getEightBallAutoPartFour());

        return command;
    }

    private Command getSquareCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        //reset robot pose
        resetRobotPose(command, container, trajectories.getSquare());
        //follow first trajectory and shoot
        //follow second trajectory and shoot
 
        // follow(command, container, trajectories.getEightBallAutoPartThree());
        // follow(command, container, trajectories.getEightBallAutoPartFour());

        return command;
    }

    private Command get8BallCompatibleCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        //reset robot pose
        resetRobotPose(command, container, trajectories.getEightBallCompatiblePartOne());
        //follow first trajectory and shoot
        follow(command, container, trajectories.getEightBallCompatiblePartOne());
 
        follow(command, container, trajectories.getEightBallCompatiblePartThree());
        follow(command, container, trajectories.getEightBallCompatiblePartFour());

        return command;
    }

    public Command getCircuit10BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // Reset the robot pose
        resetRobotPose(command, container, trajectories.getCircuitTenBallAutoPartOne());

        return command;
    }

    public Command getSimpleShootThreeAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getSimpleShootThree());

        follow(command, container, trajectories.getSimpleShootThree());

        return command;
    }

    public Command get_sCurve(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.get_sCurve());

        follow(command, container, trajectories.get_sCurve());

        return command;
    }

    public Command get_tarmacPosition1ToBall2(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.get_StartPosition1ToBall1());

        follow(command, container, trajectories.get_StartPosition1ToBall1());

        return command;
    }

    public Command get_tarmacPosition2ToBall3(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.get_StartPosition1ToBall3());

        follow(command, container, trajectories.get_StartPosition1ToBall3());

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case EIGHT_BALL:
                return get8BallAutoCommand(container);
            case EIGHT_BALL_COMPATIBLE:
                return get8BallCompatibleCommand(container);
            case TEN_BALL:
                return get10BallAutoCommand(container);
            case TEN_BALL_CIRCUIT:
                return getCircuit10BallAutoCommand(container);
            case SIMPLE_SHOOT_THREE:
                return getSimpleShootThreeAutoCommand(container);
            case SIMPLE_SQUARE:
                return getSimpleSquareCommand(container);
            case SEVEN_FEET:
                return getSevenFeet(container);
            case S_CURVE:
                return get_sCurve(container);
            case MidPositionFourBall:
                return new MidPositionFourBall(container, trajectories);
            case TerminalThreeBall:
                return new TerminalThreeBall(container, trajectories);
            case TerminalTwoBall:
                return new TerminalTwoBall(container, trajectories); 
            case TerminalFiveBall:
                return new TerminalFiveBall(container, trajectories);   
            case HangarFourBall:
                return new HangarFourBall(container, trajectories);
            case HANGAR_TWO_BALL:
                return new HangarTwoBall(container, trajectories);
            default:
                return getSevenFeet(container);
        }
        //return get10BallAutoCommand(container);
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(trajectory.calculate(0.0).getPathState().getRotation())));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), trajectory.calculate(0.0).getPathState().getRotation()))));
    }

    private enum AutonomousMode {
        EIGHT_BALL,
        EIGHT_BALL_COMPATIBLE,
        TEN_BALL,
        TEN_BALL_CIRCUIT,
        SIMPLE_SHOOT_THREE,
        SIMPLE_SQUARE, 
        SEVEN_FEET, 
        S_CURVE,
        MidPositionFourBall,
        TerminalThreeBall,
        TerminalTwoBall,
        TerminalFiveBall,
        HangarFourBall,
        HANGAR_TWO_BALL
    }
}
