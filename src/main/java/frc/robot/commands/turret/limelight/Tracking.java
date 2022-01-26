package frc.robot.commands.turret.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**TO DO
 * Tracking is a command for the turret using the limelight to follow the target as long as it can see it
 * if it reaches the edge, it exits to wrapping
 * if it loses the target, it goes to searching
 */

public class Tracking extends CommandBase {

    //needed subsystems for our command
    private final VisionSubsystem visionSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    //PID we use to calculate the velocity we should give to our turret and gives in volts
    private PIDController turretTrackingPIDController = new PIDController(0, 0, 0);

    //flag boolean
    private boolean isFinished = false;

    private double tolerance = 0.025;

    //constructor
    public Tracking(VisionSubsystem vision, ShooterSubsystem shooter) {

        visionSubsystem = vision;
        shooterSubsystem = shooter;

        //subsystems that are required to run this method
        addRequirements(vision, shooter);
        
    }

    @Override
    public void execute() {
        
        if (visionSubsystem.hasValidTarget() && shooterSubsystem.isWithinEdges()) {
            
            //calculates velocity(voltage) we should give to the motor controller
            double outputVoltage = turretTrackingPIDController.calculate(
                visionSubsystem.gettX(), tolerance); 
            shooterSubsystem.runTurretVoltage(outputVoltage);

            //puts if is in tolerance 
            SmartDashboard.putBoolean("Locked onto Target?", (visionSubsystem.gettX() < tolerance));

        } else if (!visionSubsystem.hasValidTarget()) { 
            
            //if can't see target, go to Searching
            isFinished = true;
            new Searching(visionSubsystem, shooterSubsystem).schedule();
            
        } else if (!shooterSubsystem.isWithinEdges()) {
            
            //if we reach the edge of turret range, go to Wrapping
            isFinished = true;
            new Wrapping(visionSubsystem, shooterSubsystem).schedule();
            
        }
        
    }

    @Override
    public boolean isFinished() {

        return isFinished;
    
    }
    
    /* 
    This might work ¯\_( ͡° ͜ʖ ͡°)_/¯
    We are so cool 
    
    CONTROLS TEAM IS DA COOLEST BRO
    DESIGN DOES NOT SUCKSSSSS
    */
    
}