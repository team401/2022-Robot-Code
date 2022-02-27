package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationArmSubsystem;

public class UpdateRotationArm extends CommandBase {

    private final RotationArmSubsystem rotation;

    private final double desiredPositionRadians;
    
    public UpdateRotationArm(RotationArmSubsystem climb, double desired, Constraints constraints) {

        rotation = climb;
        rotation.changePIDConstraints(constraints);
        desiredPositionRadians = desired;

        addRequirements(rotation);

    }

    @Override
    public void initialize() {
        rotation.resetControllers();
    }

    @Override
    public void execute() {

        rotation.setLeftDesiredPosition(desiredPositionRadians);
        rotation.setRightDesiredPosition(desiredPositionRadians);
        
    }

    @Override
    public boolean isFinished() {
        return rotation.atGoal() || !rotation.withinBoundaries();
    }
    
}
