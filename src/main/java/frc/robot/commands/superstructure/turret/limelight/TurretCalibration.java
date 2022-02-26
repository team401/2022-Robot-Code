package frc.robot.commands.superstructure.turret.limelight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCalibration extends CommandBase {

    private final TurretSubsystem turretSubsystem;

    private Timer turretTimer = new Timer();

    public TurretCalibration(TurretSubsystem turret) {

        turretSubsystem = turret;

        addRequirements(turret);

    }

    @Override
    public void initialize() {

        turretTimer.reset();
        turretTimer.start();

        turretSubsystem.runTurretPercent(0.2);

    }

    @Override
    public void execute() {

        if (turretSubsystem.getTurretVelocityRadPerSec() > 0.01) {

            turretTimer.reset();

        }

    }

    @Override
    public boolean isFinished() {

        return turretTimer.get() >= 0.1;

    }

    @Override
    public void end(boolean isFinished) {

        turretSubsystem.runTurretPercent(0);
        turretSubsystem.setTurretEncoderRightExtrema();

    }
    
}
