package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class HoldPositionRotationArms extends CommandBase {

    private double desiredPositionLeft;
    private double desiredPositionRight;
    private ClimbSubsystem climber;

    public HoldPositionRotationArms(ClimbSubsystem climbingSubsystem) {

        climber = climbingSubsystem;
        
        addRequirements(climbingSubsystem);

    }

    @Override
    public void initialize() {
        climber.resetRotationControllers();
        desiredPositionLeft = climber.getLeftRotationEncoderValue();
        desiredPositionRight = climber.getRightRotationEncoderValue();
    }
    
    @Override
    public void execute() {

        climber.setLeftDesiredRotationPosition(desiredPositionLeft);
        climber.setRightDesiredRotationPosition(desiredPositionRight);
        
    }

}
