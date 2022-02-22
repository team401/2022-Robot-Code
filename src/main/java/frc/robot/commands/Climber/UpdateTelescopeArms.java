package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class UpdateTelescopeArms extends CommandBase {

    private final ClimbSubsystem climbingSubsystem;
    private double position;

    public UpdateTelescopeArms(ClimbSubsystem climber, double desiredPosition) {
        climbingSubsystem = climber;
        position = desiredPosition;

        addRequirements(climbingSubsystem);
    }

    @Override
    public void execute() {

        climbingSubsystem.setLeftDesiredTelescopePosition(position);
        climbingSubsystem.setRightDesiredTelescopePosition(position);

        //climbingSubsystem.setLeftTelescopePercent(0.5);
        //climbingSubsystem.setRightTelescopePercent(0.5);

    }

    @Override
    public boolean isFinished() {
        // TODO: change velocity values
        return climbingSubsystem.atGoalTelescope() || !climbingSubsystem.withinBoundariesTelescope();
    }
    
}
 