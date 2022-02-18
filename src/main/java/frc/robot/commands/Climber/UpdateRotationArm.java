package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class UpdateRotationArm extends CommandBase {

    private final ClimbSubsystem climbingSubsystem;

    private final double desiredPositionRadians;
    
    public UpdateRotationArm(ClimbSubsystem climber, double desired) {

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

        /*climbingSubsystem.setLeftDesiredRotationPosition(desiredPositionRadians);
        climbingSubsystem.setRightDesiredRotationPosition(desiredPositionRadians);*/

        climbingSubsystem.setLeftRotationPercent(0.45);
        climbingSubsystem.setRightRotationPercent(0.45);

        
    }

    @Override
    public void end(boolean isInterrupted) {
        climbingSubsystem.setLeftRotationPercent(0);
        climbingSubsystem.setRightRotationPercent(0);

        climbingSubsystem.resetControllers();
    }

    @Override
    public boolean isFinished() {
        return false;//climbingSubsystem.atGoal() || !climbingSubsystem.withinBoundaries();
    }
    
}
