package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeArmSubsystem;

public class HoldPositionTelescopeArms extends CommandBase {
    
    private double desiredPositionLeft;
    private double desiredPositionRight;
    private TelescopeArmSubsystem telescope;

    public HoldPositionTelescopeArms(TelescopeArmSubsystem laTele) {

        telescope = laTele;
        
        addRequirements(laTele);

    }

    @Override
    public void initialize() {
        telescope.resetControllers();
        desiredPositionLeft = telescope.getLeftEncoderValue();
        desiredPositionRight = telescope.getRightEncoderValue();
    }
    
    @Override
    public void execute() {

        telescope.setLeftDesiredPosition(desiredPositionLeft);
        telescope.setRightDesiredPosition(desiredPositionRight);
        
    }

}
