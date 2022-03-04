package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutoTrajectories {

    /**
     * Defines the trajectories to be run through auto
     * 
     * Full field length: 650 inches
     * Full field width: 324 inches
     * 
     * (0,0) is the bottom left corner of the field when you are looking at it horizontally, 
     * and blue drivers station is on the left
     * 
     * All of these positions assume that our robot is centered in the middle of the pentagon
     */

    // configures the maximum velocity and accel for the trajectories
    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.maxVelocityMetersPerSec, 
            AutoConstants.maxAccelerationMetersPerSecondPerSecond
        )
        .setKinematics(DriveConstants.kinematics);

    public static Trajectory autoTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(-55), Units.inchesToMeters(0), new Rotation2d(0))
            ), 
            config
    );

}
