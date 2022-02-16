package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

public class FirstBarClimb extends CommandBase {

    private final ClimbingSubsystem climbingSubsystem;
    
    public FirstBarClimb(ClimbingSubsystem climber) {

        climbingSubsystem = climber;

        addRequirements(climbingSubsystem);

    }

    @Override
    public void periodic() {
        
    }
    
}
