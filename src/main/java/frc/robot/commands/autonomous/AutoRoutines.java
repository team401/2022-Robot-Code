package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTrajectories;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.RunAtPercent;
import frc.robot.commands.superstructure.shooting.PrepareToShoot;
import frc.robot.commands.superstructure.shooting.Shoot;
import frc.robot.commands.superstructure.shooting.HoodCalibrate;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoRoutines extends SequentialCommandGroup {

    public AutoRoutines(
        DriveSubsystem drive, 
        IntakeSubsystem intake, 
        IndexingSubsystem indexer, 
        LimelightSubsystem limelight, 
        ShooterSubsystem shooter
    ) {

        drive.resetPose(AutoTrajectories.autoTrajectory.getInitialPose());

        addCommands(
            new HoodCalibrate(shooter).withTimeout(2),
            new PrepareToShoot(shooter, limelight, 2400, 1).withTimeout(3),
            new Shoot(indexer).withTimeout(3),
            new RunAtPercent(drive).withTimeout(2.5)
        );

    }
}
