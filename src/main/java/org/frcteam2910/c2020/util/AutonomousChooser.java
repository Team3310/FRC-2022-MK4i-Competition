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
        autonomousModeChooser.addOption("Mid Position Two Ball", AutonomousMode.MidPositionTwoBall);
        autonomousModeChooser.addOption("Terminal Three Ball", AutonomousMode.TerminalThreeBall);
        autonomousModeChooser.addOption("Terminal Two Ball", AutonomousMode.TerminalTwoBall);
        autonomousModeChooser.addOption("Hangar Four Ball Steal", AutonomousMode.HANGAR_FOUR_BALL_STEAL);
        autonomousModeChooser.addOption("Hangar Four Ball", AutonomousMode.HangarFourBall);
        autonomousModeChooser.addOption("Hangar Two Ball", AutonomousMode.HANGAR_TWO_BALL);
        autonomousModeChooser.addOption("Hangar Three Ball", AutonomousMode.HANGAR_THREE_BALL);
        autonomousModeChooser.addOption("Hangar Two Ball Steal One", AutonomousMode.HANGAR_TWO_BALL_STEAL_ONE);
        autonomousModeChooser.addOption("Hangar Two Ball Steal Two", AutonomousMode.HANGAR_TWO_BALL_STEAL_TWO);
        autonomousModeChooser.addOption("Terminal Two Ball Steal Two", AutonomousMode.TERMINAL_TWO_BALL_STEAL_TWO);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    private Command getSevenFeet(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getSevenFeet());

        follow(command, container, trajectories.getSevenFeet());

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
            case SEVEN_FEET:
                return getSevenFeet(container);
            case S_CURVE:
                return get_sCurve(container);
            case MidPositionFourBall:
                return new MidPositionFourBall(container, trajectories);
            case MidPositionTwoBall:
                return new MidPositionTwoBall(container, trajectories);    
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
            case HANGAR_THREE_BALL:
                return new HangarThreeBall(container, trajectories);
            case HANGAR_FOUR_BALL_STEAL:
                return new HangarFourBallSteal(container, trajectories);
            case HANGAR_TWO_BALL_STEAL_ONE:
                return new HangarTwoBallOneBallSteal(container, trajectories);
            case HANGAR_TWO_BALL_STEAL_TWO:
                return new HangarTwoBallTwoBallSteal(container, trajectories);
            case TERMINAL_TWO_BALL_STEAL_TWO:
                return new TerminalTwoBallTwoBallSteal(container, trajectories);
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
        SEVEN_FEET, 
        S_CURVE,
        MidPositionFourBall,
        TerminalThreeBall,
        TerminalTwoBall,
        TerminalFiveBall,
        HangarFourBall,
        HANGAR_FOUR_BALL_STEAL,
        HANGAR_TWO_BALL,
        HANGAR_THREE_BALL,
        MidPositionTwoBall,
        HANGAR_TWO_BALL_STEAL_ONE,
        HANGAR_TWO_BALL_STEAL_TWO,
        TERMINAL_TWO_BALL_STEAL_TWO
        ;
    }
}
