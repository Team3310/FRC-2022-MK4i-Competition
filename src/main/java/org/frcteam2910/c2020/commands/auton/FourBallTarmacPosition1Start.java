package org.frcteam2910.c2020.commands.auton;

import com.fasterxml.jackson.databind.cfg.ContextAttributes;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.FeedBalls;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.HoodSetAngle;
import org.frcteam2910.c2020.commands.IndexerBallStop;
import org.frcteam2910.c2020.commands.IndexerSetRPM;
import org.frcteam2910.c2020.commands.IntakeIndexerHalt;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.commands.ShooterSetRPM;
import org.frcteam2910.c2020.commands.ShooterShootWithHood;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourBallTarmacPosition1Start extends AutonCommandBase {
    public FourBallTarmacPosition1Start(RobotContainer container, AutonomousTrajectories trajectories) {
        resetRobotPose(container, trajectories.get_tarmacPosition1ToBall2());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        this.addCommands(
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new WaitCommand(.5),
            new ShooterShootWithHood(container.getShooter(), 2100, 32),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_tarmacPosition1ToBall2()),
            new ChangeDriveMode(container.getDrivetrainSubsystem(), DriveControlMode.LIMELIGHT),
            new WaitCommand(.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(.5),
            new ChangeDriveMode(container.getDrivetrainSubsystem(), DriveControlMode.TRAJECTORY),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer()),
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_tarmacPosition2ToBall3()),
            new WaitCommand(.75),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_tarmacPosition3ToBall2()),
            new WaitCommand(.75),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_tarmacPosition4ToPosition3()),
            new WaitCommand(.75),
            new ShooterShootWithHood(container.getShooter(), 2100, 32),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_tarmacPosition3ToPosition2()),
            new ChangeDriveMode(container.getDrivetrainSubsystem(), DriveControlMode.LIMELIGHT),
            new WaitCommand(.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(.5),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer())
        );
    }
}
