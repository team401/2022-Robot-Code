package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class CalibrateTelescope extends CommandBase {

    private ClimbSubsystem climb;
    private Timer leftClimbTimer = new Timer();
    private Timer rightClimbTimer = new Timer();

    public CalibrateTelescope(ClimbSubsystem climber) {

        climb = climber;
        addRequirements(climber);

    }

    @Override
    public void initialize() {

        leftClimbTimer.reset();
        rightClimbTimer.reset();

        leftClimbTimer.start();
        rightClimbTimer.start();
        climb.setLeftTelescopePercent(-0.25);
        climb.setRightTelescopePercent(-0.25);

    }

    @Override
    public void execute() {

        SmartDashboard.putNumber("left telescope velocity", climb.getLeftTelescopeVelocity());
        if (Math.abs(climb.getLeftTelescopeVelocity()) > 0.2)
             leftClimbTimer.reset();
             SmartDashboard.putBoolean("left finished", false);
        if (leftClimbTimer.get() >= 0.025) {
            climb.setLeftTelescopePercent(0.0);
            SmartDashboard.putBoolean("left finished", true);
        }
        if (Math.abs(climb.getRightTelescopeVelocity()) > 0.2) 
            rightClimbTimer.reset();
            SmartDashboard.putBoolean("right finished", false);
        if (rightClimbTimer.get() >= 0.025) {
            climb.setRightTelescopePercent(0.0);
            SmartDashboard.putBoolean("right finished", true);
        }
        SmartDashboard.putNumber("left timer", leftClimbTimer.get());
        
    }

    @Override
    public boolean isFinished() {

        return leftClimbTimer.get() >= 0.025 && rightClimbTimer.get() >= 0.025;

    }

    @Override
    public void end(boolean isInterrupted) {

        climb.setLeftTelescopePercent(0);
        climb.setRightTelescopePercent(0);
        //TODO: reset encoders
        climb.resetLeftTelescopeEncoder();
        climb.resetRightTelescopeEncoder();

    }
    

    
}
