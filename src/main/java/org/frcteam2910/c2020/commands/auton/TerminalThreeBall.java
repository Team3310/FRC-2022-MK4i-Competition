package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.AllFieldAuton;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.FeedBalls;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.IndexerBallStop;
import org.frcteam2910.c2020.commands.IntakeIndexerHalt;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.subsystems.*;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TerminalThreeBall extends AutonCommandBase {

    public TerminalThreeBall(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public TerminalThreeBall(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.get_StartPosition0ToBall1());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        this.addCommands(
            new TerminalTwoBall(container, trajectories),
                new WaitCommand(0.5),
                new ChangeDriveMode(drive, DriveControlMode.TRAJECTORY),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.get_ThreeBallPartTwo()),
                        new IndexerBallStop(indexer)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.0),
                        new AllFieldAuton(shooter, drive)
                ),
                new FeedBalls(intake, indexer)
        );
    }
}
