package frc.robot.commands.superstructure.turret.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**TO DO
 * Tracking is a command for the turret using the limelight to follow the target as long as it can see it
 * if it reaches the edge, it exits to wrapping
 * if it loses the target, it goes to searching
 */
public class Tracking extends CommandBase {
    
    // Subystems
    private final VisionSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;

    //PID we use to calculate the velocity we should give to our turret and gives in volts
    private final PIDController turretTrackingPIDController = new PIDController(1.2, 0.5, 0);

    // Flag boolean
    private boolean isFinished = false;

    // Constructor
    public Tracking(VisionSubsystem limelight, TurretSubsystem turret) {

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

        SmartDashboard.putBoolean("Tracking Running", true);

        if (limelightSubsystem.hasValidTarget() && turretSubsystem.isWithinEdges()) {

            SmartDashboard.putNumber("Reached here:", System.currentTimeMillis());
            // Calculates output we should give to the motor controller
            //double output = turretTrackingPIDController.calculate(limelightSubsystem.gettX(), 0); 

            //limelight tx value [-27,27] converted to radians 
            double limelightErrorRadians = Units.degreesToRadians(-limelightSubsystem.gettX());

            // If the error is more than 0.1, set the desired position to be 0.1 closer to target
            if (Math.abs(limelightErrorRadians) >= 0.1) { 

                SmartDashboard.putBoolean("SetDesiredPosition ++ 1", true);

                turretSubsystem.setTurretDesiredClosedState(
                    turretSubsystem.getTurretPositionRadians() + (Math.signum(limelightErrorRadians) * 0.1));
                    
            } else if (Math.abs(limelightErrorRadians) < 0.1) {
            //If error is within 0.1, set the desired position to be the error

                turretSubsystem.setTurretDesiredClosedState(
                    turretSubsystem.getTurretPositionRadians() + limelightErrorRadians);

            }

        } else {

            SmartDashboard.putBoolean("Tracking Running", false);

            // Finish tracking and start wrapping
            isFinished = true;
            //true sets command to interruptable 
            new BasicSearch(limelightSubsystem, turretSubsystem).schedule(true);

        }

    }

    @Override
    public boolean isFinished() {

        // Finish wrapping if tracking has started or the turret subsystem is set to stop tracking
        return isFinished; //|| !turretSubsystem.shouldBeTracking();

    }

    @Override
    public void end(boolean isInterrupted) {

        // Stop the motor from spinning after we end the command
        turretSubsystem.runTurretPercent(0);

        SmartDashboard.putBoolean("Tracking Running", false);

    }

}

    /* 
    This might work ¯\_( ͡° ͜ʖ ͡°)_/¯
    We are so cool 
    
    CONTROLS TEAM IS DA COOLEST BRO
    DESIGN DOES NOT SUCKSSSSS
    */
    