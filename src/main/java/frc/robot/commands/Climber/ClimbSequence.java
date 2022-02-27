package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RotationArmSubsystem;
import frc.robot.subsystems.TelescopeArmSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ClimbSequence extends SequentialCommandGroup {

    public ClimbSequence(TelescopeArmSubsystem telescope, RotationArmSubsystem rotation, TurretSubsystem turret) {

        addCommands(
            new UpdateTelescopeArms(telescope, turret, 28),
            new UpdateTelescopeArms(telescope, turret, 1),
            new UpdateRotationArm(
                rotation, 
                Units.degreesToRadians(15), 
                new TrapezoidProfile.Constraints(10.0, 15.0)
            ),
            new InstantCommand(() -> telescope.setLeftPercent(0)),
            new InstantCommand(() -> telescope.setRightPercent(0))
 
        );

    }
    
}
