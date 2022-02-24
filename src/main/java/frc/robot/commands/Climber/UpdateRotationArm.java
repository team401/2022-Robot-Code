package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class UpdateRotationArm extends CommandBase {

    private final ClimbSubsystem climbingSubsystem;

    private final double desiredPositionRadians;
    
    public UpdateRotationArm(ClimbSubsystem climber, double desired, Constraints constraints) {

        climbingSubsystem = climber;
        climbingSubsystem.changeRotationPIDConstraints(constraints);
        desiredPositionRadians = desired;

        addRequirements(climbingSubsystem);

    }

    @Override
    public void initialize() {
        climbingSubsystem.resetRotationControllers();
    }

    @Override
    public void execute() {

        climbingSubsystem.setLeftDesiredRotationPosition(desiredPositionRadians);
        climbingSubsystem.setRightDesiredRotationPosition(desiredPositionRadians);

        SmartDashboard.putNumber("left rot", desiredPositionRadians);
        SmartDashboard.putNumber("right rot", desiredPositionRadians);

        //climbingSubsystem.setLeftRotationPercent(-0.15);
        //climbingSubsystem.setRightRotationPercent(-0.15);
        
    }

    @Override
    public boolean isFinished() {
        return climbingSubsystem.atGoalRotation() || !climbingSubsystem.withinBoundariesRotation();
    }
    
}
