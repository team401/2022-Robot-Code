package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeArmSubsystem;

public class CalibrateTelescope extends CommandBase {

    private TelescopeArmSubsystem telescope;
    private Timer leftTimer = new Timer();
    private Timer rightTimer = new Timer();
    private Timer resetTimer = new Timer();

    public CalibrateTelescope(TelescopeArmSubsystem climber) {

        telescope = climber;
        addRequirements(climber);

    }

    @Override
    public void initialize() {

        leftTimer.reset();
        rightTimer.reset();
        resetTimer.reset();

        leftTimer.start();
        rightTimer.start();
        resetTimer.stop();

        telescope.setLeftPercent(-0.4);
        telescope.setRightPercent(-0.4);

    }

    @Override
    public void execute() {

        if (Math.abs(telescope.getLeftVelocity()) > 0.2) {
            leftTimer.reset();
        }
        if (Math.abs(telescope.getRightVelocity()) > 0.2) {
            rightTimer.reset();
        }

        if (leftTimer.get() >= 0.05) {
            telescope.setLeftPercent(0.0);
        }
        if (rightTimer.get() >= 0.05) {
            telescope.setRightPercent(0.0);
        }

        if (leftTimer.get() >= 0.05 && rightTimer.get() >= 0.05) {
            resetTimer.start();
        }

    }

    @Override
    public boolean isFinished() {

        return resetTimer.get() >= 0.75;

    }

    @Override
    public void end(boolean isInterrupted) {

        telescope.setLeftPercent(0);
        telescope.setRightPercent(0);

        telescope.resetLeftEncoder();
        telescope.resetRightEncoder();

    }
    

    
}
