package frc.robot.commands.superstructure.turret.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**TO DO
 * Tracking is a command for the turret using the limelight to follow the target as long as it can see it
 * if it reaches the edge, it exits to wrapping
 * if it loses the target, it goes to searching
 */
public class Tracking extends CommandBase {
    
    // Subystems
    private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;

    // Constructor
    public Tracking(LimelightSubsystem limelight, TurretSubsystem turret) {

        // Set the subsystems
        limelightSubsystem = limelight;
        turretSubsystem = turret;

        // Use the limelight and turret subsystems
        addRequirements(limelightSubsystem, turretSubsystem);

    }

    @Override
    public void initialize() {

        SmartDashboard.putBoolean("Tracking Running", true);

    }

    @Override
    public void execute() {

        if (limelightSubsystem.hasValidTarget() && turretSubsystem.isWithinEdges()) {

            // Calculates output we should give to the motor controller
            //double output = turretTrackingPIDController.calculate(limelightSubsystem.gettX(), 0); 

            //limelight tx value [-27,27] converted to radians 
            double limelightErrorRadians = Units.degreesToRadians(-limelightSubsystem.getX());

            // If the error is more than 0.1, set the desired position to be 0.1 closer to target
            if (Math.abs(limelightErrorRadians) >= 0.1) { 

                turretSubsystem.setTurretDesiredClosedState(
                    turretSubsystem.getTurretPositionRadians() + (Math.signum(limelightErrorRadians) * 0.1));
                    
            } else if (Math.abs(limelightErrorRadians) < 0.1) {
            //If error is within 0.1, set the desired position to be the error

                turretSubsystem.setTurretDesiredClosedState(
                    turretSubsystem.getTurretPositionRadians() + limelightErrorRadians);

            }

        } else {

            // Stop moving the turret
            turretSubsystem.runTurretPercent(0);
            // True sets command to interruptable 
            new BasicSearch(limelightSubsystem, turretSubsystem).schedule(true);
            // Update SmartDashboard
            SmartDashboard.putBoolean("Tracking Running", false);

        }

    }

}