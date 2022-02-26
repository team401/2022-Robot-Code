package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class CalibrateTelescope extends CommandBase {

    private ClimbSubsystem climb;
    private Timer leftClimbTimer = new Timer();
    private Timer rightClimbTimer = new Timer();
    private Timer resetTimer = new Timer();

    public CalibrateTelescope(ClimbSubsystem climber) {

        climb = climber;
        addRequirements(climber);

    }

    @Override
    public void initialize() {

        leftClimbTimer.reset();
        rightClimbTimer.reset();
        resetTimer.reset();

        leftClimbTimer.start();
        rightClimbTimer.start();
        resetTimer.stop();
        climb.setLeftTelescopePercent(-0.25);
        climb.setRightTelescopePercent(-0.25);

    }

    @Override
    public void execute() {

        SmartDashboard.putNumber("left telescope velocity", climb.getLeftTelescopeVelocity());
        SmartDashboard.putNumber("right telescope velocity", climb.getRightTelescopeVelocity());

        if (Math.abs(climb.getLeftTelescopeVelocity()) > 0.2) {
            leftClimbTimer.reset();
            SmartDashboard.putBoolean("left finished", false);
        }
        if (Math.abs(climb.getRightTelescopeVelocity()) > 0.2) {
            rightClimbTimer.reset();
            SmartDashboard.putBoolean("right finished", false);
        }

        if (leftClimbTimer.get() >= 0.05) {
            climb.setLeftTelescopePercent(0.0);
            SmartDashboard.putBoolean("left finished", true);
        }
        if (rightClimbTimer.get() >= 0.05) {
            climb.setRightTelescopePercent(0.0);
            SmartDashboard.putBoolean("right finished", true);
        }
        

        SmartDashboard.putNumber("left timer", leftClimbTimer.get());
        SmartDashboard.putNumber("right timer", rightClimbTimer.get());

        if (leftClimbTimer.get() >= 0.05 && rightClimbTimer.get() >= 0.05) {
            resetTimer.start();
        }

    }

    @Override
    public boolean isFinished() {

        return resetTimer.get() >= 0.75;
        //return leftClimbTimer.get() >= 0.05 && rightClimbTimer.get() >= 0.05;

    }

    @Override
    public void end(boolean isInterrupted) {

        SmartDashboard.putNumber("DONE TIME!!", System.currentTimeMillis());

        climb.setLeftTelescopePercent(0);
        climb.setRightTelescopePercent(0);

        climb.resetLeftTelescopeEncoder();
        climb.resetRightTelescopeEncoder();

    }
    

    
}
