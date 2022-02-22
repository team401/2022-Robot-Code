package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class HoldPositionTelescopeArms extends CommandBase {
    
    private double desiredPositionLeft;
    private double desiredPositionRight;
    private ClimbSubsystem climber;

    public HoldPositionTelescopeArms(ClimbSubsystem climbingSubsystem) {

        climber = climbingSubsystem;
        
        addRequirements(climbingSubsystem);

    }

    @Override
    public void initialize() {
        climber.resetControllers();
        desiredPositionLeft = climber.getLeftTelescopeEncoderValue();
        desiredPositionRight = climber.getRightTelescopeEncoderValue();
    }
    
    @Override
    public void execute() {

        climber.setLeftDesiredTelescopePosition(desiredPositionLeft);
        climber.setRightDesiredTelescopePosition(desiredPositionRight);
        
    }

}
