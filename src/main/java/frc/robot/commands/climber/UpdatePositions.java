package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class UpdatePositions extends CommandBase {

    private final ClimbSubsystem climbingSubsystem;

    private final double desiredPositionRadians;
    
    public UpdatePositions(ClimbSubsystem climber, double desired) {

        climbingSubsystem = climber;

        desiredPositionRadians = desired;

        addRequirements(climbingSubsystem);

    }

    @Override
    public void initialize() {
        climbingSubsystem.resetControllers();
    }

    @Override
    public void execute() {

        climbingSubsystem.setLeftDesiredRotationPosition(desiredPositionRadians);
        climbingSubsystem.setRightDesiredRotationPosition(desiredPositionRadians);
        
    }

    @Override
    public void end(boolean isInterrupted) {
        climbingSubsystem.setLeftRotationPercent(0);
        climbingSubsystem.setRightRotationPercent(0);
    }

    @Override
    public boolean isFinished() {
        return climbingSubsystem.atGoal() || !climbingSubsystem.withinBoundaries();
    }
    
}
