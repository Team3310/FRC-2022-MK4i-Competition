package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.AllFieldAuton;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.FeedBalls;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.IndexerBallStop;
import org.frcteam2910.c2020.commands.IntakeIndexerHalt;
import org.frcteam2910.c2020.commands.ShooterShootAllFieldAuto;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.subsystems.*;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class MidPositionFourBall extends AutonCommandBase {

    public MidPositionFourBall(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public MidPositionFourBall(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {
        this.addCommands(
                new MidPositionTwoBall(container, trajectories),
                new WaitCommand(0.8),
                new ChangeDriveMode(drive, DriveControlMode.TRAJECTORY),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.get_MidPositionFourBallPart2()),
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
                        new WaitCommand(0.5),
                        new AllFieldAuton(shooter, drive)
                ),
                new FeedBalls(intake, indexer, drive, shooter, Constants.AUTON_INDEXER_RPM)
//                new WaitCommand(0.5),
//                new IntakeIndexerHalt(intake, indexer)
        );
    }
}
