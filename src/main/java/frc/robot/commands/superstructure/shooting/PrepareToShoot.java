package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class PrepareToShoot extends CommandBase {
    
    private final ShooterSubsystem shooterSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    private double defaultRPM;

    public PrepareToShoot(ShooterSubsystem shooter, LimelightSubsystem limelight, double defaultSpeedRPM) {

        shooterSubsystem = shooter;
        limelightSubsystem = limelight;

        defaultRPM = defaultSpeedRPM;

        SmartDashboard.putNumber("Hood Positions Revolutions", 0);
        SmartDashboard.putNumber("RPM", 0);
        

        addRequirements(shooter, limelight);

    }

    @Override
    public void execute() {

        SmartDashboard.getNumber("Hood Revolution", 0);
        SmartDashboard.getNumber("RPM", 0);


        if (limelightSubsystem.hasValidTarget()) {
            double limelightVerticalOffset = limelightSubsystem.getY();

            double desiredHoodPosition = 0; //regression goes here 
            double calculatedRPM = 0; //regression part 2

            //shooterSubsystem.hoodSetDesiredClosedStateRevolutions(desiredHoodPosition);
            shooterSubsystem.runShooterVelocityController(calculatedRPM);
        } else {

            shooterSubsystem.runShooterVelocityController(defaultRPM);

        }

    }

    @Override
    public void end(boolean isInterrupted) {
        shooterSubsystem.stopShooter();
    }
    
}