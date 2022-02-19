package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ExtendTelescope extends CommandBase {

    private final ClimbSubsystem climbingSubsystem;

    public ExtendTelescope(ClimbSubsystem climber) {
        climbingSubsystem = climber;

        addRequirements(climbingSubsystem);
    }

    @Override
    public void execute() {

        climbingSubsystem.setLeftTelescopePercent(0.5);
        climbingSubsystem.setRightTelescopePercent(0.5);

    }

    @Override
    public void end(boolean isInterrupted) {
        climbingSubsystem.setLeftTelescopePercent(0);
        climbingSubsystem.setRightTelescopePercent(0);
    }

    @Override
    public boolean isFinished() {
        // TODO: change velocity values
        return climbingSubsystem.getLeftTelescopeEncoderValue() >= ClimberConstants.leftTelescopeMaxHeight || 
                climbingSubsystem.getRightTelescopeEncoderValue() >= ClimberConstants.rightTelescopeMaxHeight ||
                Math.abs(climbingSubsystem.getLeftTelescopeVelocity()) < 0.025 || 
                Math.abs(climbingSubsystem.getRightTelescopeVelocity()) < 0.025;
    }
    
}
 