package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HangarFourBall extends AutonCommandBase {

    public HangarFourBall(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public HangarFourBall(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {

        addCommands(
                new HangarTwoBall(container, trajectories),
                new WaitCommand(0.5),
                new ChangeDriveMode(drive, DriveControlMode.TRAJECTORY),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.get_HangarFourBallPartTwo()),
                        new IndexerBallStop(indexer)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(2.0),
                        new FollowTrajectoryCommand(drive, trajectories.get_TerminalToLoadPosition()),
                        new IndexerBallStop(indexer)
                ),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.get_LoadToShootPosition()),
                        new ShooterShootAllFieldAuto(shooter)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.0),
                        new AllFieldAuton(shooter, drive)
                ),
                new FeedBalls(intake, indexer, Constants.AUTON_INDEXER_RPM),
                new WaitCommand(0.5),
                new IntakeIndexerHalt(intake, indexer)
        );
    }
}
