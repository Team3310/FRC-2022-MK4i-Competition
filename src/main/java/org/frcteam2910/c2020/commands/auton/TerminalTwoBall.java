package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TerminalTwoBall extends AutonCommandBase {
    public TerminalTwoBall(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getShooter(), container.getIndexer(), container.getIntakeSubsystem(), container.getDrivetrainSubsystem());
    }

    public TerminalTwoBall(RobotContainer container, AutonomousTrajectories trajectories, Shooter shooter, Indexer indexer, Intake intake, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.get_StartPosition0ToBall1());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        this.addCommands(
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drive, trajectories.get_StartPosition0ToBall1()),
                        new ShooterShootWithHoodAuton(shooter, 1950, 30.7),
                        new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_AUTO_RPM)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new LimelightAdjustAuto(drive)
                ),
                new FeedBalls(intake, indexer, Constants.AUTON_INDEXER_RPM)
        );
    }
}
