package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class RetractTelescope extends CommandBase {

    private final ClimbSubsystem climbingSubsystem;

    public RetractTelescope(ClimbSubsystem climber) {
        climbingSubsystem = climber;

        addRequirements(climbingSubsystem);
    }

    @Override
    public void execute() {

        climbingSubsystem.setLeftTelescopePercent(-0.1);
        climbingSubsystem.setRightTelescopePercent(-0.1);

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
                Math.abs(climbingSubsystem.getLeftTelescopeVelocity()) < 0.05 || 
                Math.abs(climbingSubsystem.getRightTelescopeVelocity()) < 0.05;
    }
    
}
 