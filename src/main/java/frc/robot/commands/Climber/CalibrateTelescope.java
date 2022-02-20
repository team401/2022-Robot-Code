package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class CalibrateTelescope extends CommandBase {

    private ClimbSubsystem climb = new ClimbSubsystem();
    private Timer leftClimbTimer = new Timer();
    private Timer rightClimbTimer = new Timer();

    public CalibrateTelescope(ClimbSubsystem climber) {

        climb = climber;
        addRequirements(climber);

    }

    @Override
    public void initialize() {

        leftClimbTimer.start();
        rightClimbTimer.start();
        climb.setLeftTelescopePercent(-0.05);
        climb.setRightTelescopePercent(-0.05);

    }

    @Override
    public void execute() {

        if (climb.getLeftTelescopeVelocity() > 0.0) leftClimbTimer.reset();
        if (climb.getRightTelescopeVelocity() > 0.0) rightClimbTimer.reset();
        
    }

    @Override
    public boolean isFinished() {

        return leftClimbTimer.get() >= 0.15 && rightClimbTimer.get() >= 0.15;

    }

    @Override
    public void end(boolean isInterrupted) {

        climb.setLeftTelescopePercent(0);
        climb.setRightTelescopePercent(0);
        //TODO: reseet encoders
        climb.resetLeftTelescopeEncoder();
        climb.resetRightTelescopeEncoder();

    }
    

    
}
