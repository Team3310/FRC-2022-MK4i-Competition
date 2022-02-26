package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.FeedBalls;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.IntakeIndexerHalt;
import org.frcteam2910.c2020.commands.IntakeSetRPM;
import org.frcteam2910.c2020.commands.ShooterShootAllField;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FiveBall extends AutonCommandBase {
    public FiveBall(RobotContainer container, AutonomousTrajectories trajectories) {
        resetRobotPose(container, trajectories.get_StartPosition0ToBall1());
        //follow(container, trajectories.get_tarmacPosition1ToBall2());
        this.addCommands(
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition0ToBall1()),
            new ShooterShootAllField(container.getShooter(), container.getDrivetrainSubsystem()),
            new WaitCommand(0.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(0.5),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer()),
            new ChangeDriveMode(container.getDrivetrainSubsystem(), DriveControlMode.TRAJECTORY),
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_ThreeBallPartTwo()),
            new ShooterShootAllField(container.getShooter(), container.getDrivetrainSubsystem()),
            new WaitCommand(0.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(0.5),
            new ChangeDriveMode(container.getDrivetrainSubsystem(), DriveControlMode.TRAJECTORY),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer()),
            new IntakeSetRPM(container.getIntakeSubsystem(), Constants.INTAKE_COLLECT_RPM),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToBall2()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToBall3()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.get_StartPosition1ToShoot()),
            new ShooterShootAllField(container.getShooter(), container.getDrivetrainSubsystem()),
            new WaitCommand(0.5),
            new FeedBalls(container.getIntakeSubsystem(), container.getIndexer()),
            new WaitCommand(0.5),
            new IntakeIndexerHalt(container.getIntakeSubsystem(), container.getIndexer())

        );
    }
}
