package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class PrepareToShoot extends CommandBase {
    
    private final ShooterSubsystem shooterSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    private double desiredRPM;
    private double desiredHood;

    public PrepareToShoot(ShooterSubsystem shooter, LimelightSubsystem limelight, double desiredSpeedRPM, double desiredHoodPosition) {

        shooterSubsystem = shooter;
        limelightSubsystem = limelight;

        desiredRPM = desiredSpeedRPM;
        desiredHood = desiredHoodPosition;

        addRequirements(shooter, limelight);

    }

    @Override
    public void execute() {

        shooterSubsystem.runShooterVelocityController(desiredRPM);
        shooterSubsystem.hoodSetDesiredClosedStateRevolutions(desiredHood);


        /*if (limelightSubsystem.hasValidTarget()) {
            double limelightVerticalOffset = limelightSubsystem.getY();

            double desiredHoodPosition = 0; //regression goes here 
            double calculatedRPM = 0; //regression part 2

            //shooterSubsystem.hoodSetDesiredClosedStateRevolutions(desiredHoodPosition);
            shooterSubsystem.runShooterVelocityController(calculatedRPM);
        } else {

            shooterSubsystem.runShooterVelocityController(defaultRPM);

        }*/

    }
    
}