package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RotationArmSubsystem;

public class DefaultRotationArm extends CommandBase {

    private final RotationArmSubsystem rotation;
    
    public DefaultRotationArm(RotationArmSubsystem climb) {

        rotation = climb;

        rotation.setPIDConstraints(new TrapezoidProfile.Constraints(10.0, 15.0));

        addRequirements(rotation);

    }

    @Override
    public void initialize() {
        rotation.resetControllers();
    }

    @Override
    public void execute() {

        if (rotation.withinBoundaries()) {
            rotation.setLeftDesiredPosition(ClimberConstants.defaultArmPosition);
            rotation.setRightDesiredPosition(ClimberConstants.defaultArmPosition);
        }
        
    }
    
}
