package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodCalibrate extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;

    private Timer hoodTimer = new Timer();

    private double zeroVelocityTimeStamp;
    private boolean timeStampMeasured;

    public HoodCalibrate(ShooterSubsystem shooter) {

        shooterSubsystem = shooter;

        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize() {

        hoodTimer.start();

        shooterSubsystem.runHoodPercent(-0.05);

    }

    @Override
    public void execute() {

        double hoodVelocity = shooterSubsystem.getHoodVelocity();

        if (hoodVelocity > 0)
            hoodTimer.reset();
        
    }

    @Override
    public boolean isFinished() {
        return hoodTimer.get() >= 0.2;
    }

    @Override
    public void end(boolean isInterrupted) {
        shooterSubsystem.runHoodPercent(0);
        shooterSubsystem.resetHoodEncoder();
    }
    
}
