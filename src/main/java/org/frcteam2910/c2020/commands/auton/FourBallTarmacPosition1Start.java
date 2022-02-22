package org.frcteam2910.c2020.commands.auton;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

public class FourBallTarmacPosition1Start extends AutonCommandBase {
    public FourBallTarmacPosition1Start(RobotContainer container, AutonomousTrajectories trajectories) {
        resetRobotPose(container, trajectories.get_tarmacPosition1ToBall2());
        follow(container, trajectories.get_tarmacPosition1ToBall2());
    }
}
