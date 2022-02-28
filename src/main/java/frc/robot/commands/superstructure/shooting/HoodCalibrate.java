package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodCalibrate extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;

    private Timer hoodTimer = new Timer();

    public HoodCalibrate(ShooterSubsystem shooter) {

        shooterSubsystem = shooter;

        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize() {

        hoodTimer.reset();
        hoodTimer.start();

        shooterSubsystem.runHoodPercent(-0.2);

    }

    @Override
    public void execute() {

        SmartDashboard.putNumber("Hood Velocity", shooterSubsystem.getHoodVelocity());

        if (Math.abs(shooterSubsystem.getHoodVelocity()) > 0.01) {

            hoodTimer.reset();

        }
        
    }

    @Override
    public boolean isFinished() {

        return hoodTimer.get() >= 0.1;

    }

    @Override
    public void end(boolean isInterrupted) {

        shooterSubsystem.runHoodPercent(0);
        shooterSubsystem.resetHoodEncoder();
        shooterSubsystem.setHoodSoftLimits(5, 0);
        
    }
    
}
