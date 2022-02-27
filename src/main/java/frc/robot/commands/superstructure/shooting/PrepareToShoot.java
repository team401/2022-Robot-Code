package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class PrepareToShoot extends CommandBase {
    
    private final ShooterSubsystem shooterSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    private double desiredSpeed;
    private double currentLimelightVerticalOffset;
    private double desiredHoodPosition;

    private Timer shooterTimer = new Timer();

    public PrepareToShoot(ShooterSubsystem shooter, LimelightSubsystem limelight, double desiredSpeedRPM) {

        shooterSubsystem = shooter;
        limelightSubsystem = limelight;

        desiredSpeed = desiredSpeedRPM;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        if (limelightSubsystem.hasValidTarget()) {
            currentLimelightVerticalOffset = limelightSubsystem.getY();

            desiredHoodPosition = 0; //regression goes here 

            shooterSubsystem.hoodSetDesiredClosedStateRevolutions(desiredHoodPosition);
            shooterSubsystem.runShooterVelocityController(desiredSpeed);
        }

    }

    @Override
    public boolean isFinished() {

        return shooterSubsystem.atGoal();

    }
    
}