package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationArmSubsystem;
import frc.robot.subsystems.RotationArmSubsystem.Mode;

public class UpdateRotationArm extends CommandBase {

    private final RotationArmSubsystem rotation;

    private final double desiredPositionRadians;
    
    public UpdateRotationArm(RotationArmSubsystem climb, double desired, Mode mode) {

        rotation = climb;
        /*if (mode == Mode.Climbing) {
            rotation.setPIDConstraints(new TrapezoidProfile.Constraints(.5, .5));
        }
        else if (mode == Mode.Intaking) {
            rotation.setPIDConstraints(new TrapezoidProfile.Constraints(10.0, 15.0));
        }*/
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
