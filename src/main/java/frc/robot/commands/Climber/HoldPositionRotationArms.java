package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationArmSubsystem;

public class HoldPositionRotationArms extends CommandBase {

    private double desiredPositionLeft;
    private double desiredPositionRight;
    private RotationArmSubsystem rotation;

    public HoldPositionRotationArms(RotationArmSubsystem climber) {

        rotation = climber;
        
        addRequirements(rotation);

    }

    @Override
    public void initialize() {
        rotation.resetControllers();
        desiredPositionLeft = rotation.getLeftEncoderValue();
        desiredPositionRight = rotation.getRightEncoderValue();
    }
    
    @Override
    public void execute() {

        rotation.setLeftDesiredPosition(desiredPositionLeft);
        rotation.setRightDesiredPosition(desiredPositionRight);
        
    }

}
