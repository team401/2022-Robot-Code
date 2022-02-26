package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTrajectories;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.superstructure.shooting.RampUp;
import frc.robot.commands.superstructure.shooting.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoRoutines extends SequentialCommandGroup {
    
    public enum StartingPosition {
        Left,
        Right
    }

    public enum Side {
        Blue,
        Red
    }

    private final StartingPosition startingPosition;
    private final Side side;

    public AutoRoutines(
        StartingPosition position, 
        Side sideInputed, 
        DriveSubsystem drive, 
        IntakeSubsystem intake, 
        IndexingSubsystem indexer, 
        LimelightSubsystem limelight, 
        ShooterSubsystem shooter
    ) {

        startingPosition = position;
        side = sideInputed;

        drive.resetIMU();

        //need to add the actual commands

        switch (startingPosition) {

            case Left:
                
                switch (side) {

                    case Blue:

                        drive.resetPose(AutoTrajectories.LeftBlue.getInitialPose());

                        addCommands(
                            new SequentialCommandGroup(
                                new RampUp(shooter, Units.rotationsPerMinuteToRadiansPerSecond(4100)),
                                new Shoot(shooter, indexer)
                            ),
                            new FollowTrajectory(drive, AutoTrajectories.LeftBlue)
                        );

                        break;
                    
                    case Red:
 
                        drive.resetPose(AutoTrajectories.LeftRed.getInitialPose());

                        addCommands(
                            new ParallelCommandGroup(
                                new Shoot(shooter, indexer)
                            ),
                            new FollowTrajectory(drive, AutoTrajectories.LeftRed)
                        );

                        break;

                }

            case Right:

                switch (side) {

                    case Blue:

                        drive.resetPose(AutoTrajectories.RightBlue.getInitialPose());
                        addCommands(
                            new SequentialCommandGroup(
                                new RampUp(shooter, Units.rotationsPerMinuteToRadiansPerSecond(4100)),
                                new Shoot(shooter, indexer)
                            ),
                            new FollowTrajectory(drive, AutoTrajectories.RightBlue)
                        );

                        break;
                
                    case Red:

                        drive.resetPose(AutoTrajectories.RightRed.getInitialPose());
                            addCommands(
                                new SequentialCommandGroup(
                                    new RampUp(shooter, Units.rotationsPerMinuteToRadiansPerSecond(4100)),
                                    new Shoot(shooter, indexer)
                                ),
                                new FollowTrajectory(drive, AutoTrajectories.RightRed)
                            );

                        break;
                }

            }
        }



}
