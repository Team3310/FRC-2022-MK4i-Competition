package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.subsystems.Indexer;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.subsystems.Shooter;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

public class HangarTwoBallTwoBallStealDirect extends AutonCommandBase {

    public HangarTwoBallTwoBallStealDirect(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public HangarTwoBallTwoBallStealDirect(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {

        addCommands(
                new HangarTwoBall(container, trajectories),
                new WaitCommand(0.8),
                new ChangeDriveMode(drive, DriveControlMode.TRAJECTORY),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.getHangarTwoBallStealOne()),
                        new IndexerBallStop(indexer)
                ),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.getHangarStealTwoDirect()),
                        new IndexerBallStop(indexer)
                ),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.getHangarTwoBallStealTwoPlace()),
                        new IndexerBallStop(indexer)
                ),
                new FollowTrajectoryCommand(drive, trajectories.getHangarTwoBallStealOnePlaceBackup()),
                new ParallelDeadlineGroup(
                        new WaitCommand(2.0),
                        new EjectBalls(intake, indexer, shooter)
                ),
                new IntakeIndexerHalt(intake, indexer)
        );
    }
}
