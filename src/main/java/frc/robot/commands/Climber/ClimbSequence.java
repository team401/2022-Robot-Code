package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ClimbSequence extends SequentialCommandGroup {

    public ClimbSequence(ClimbSubsystem climber, TurretSubsystem turret) {

        addCommands(
            //new UpdateTelescopeArms(climber, turret, desiredPosition)
        );

    }
    
}
