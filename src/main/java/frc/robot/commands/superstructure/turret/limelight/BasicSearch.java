package frc.robot.commands.superstructure.turret.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class BasicSearch extends CommandBase {

    // Subystems
    private final VisionSubsystem visionSubsystem;
    private final TurretSubsystem turretSubsystem;

    // Direction of the turret movement; -1 (left) 1 (Positive)
    private int direction = 0;

    // Flag boolean
    private boolean isFinished = false;
    
    // Constructor
    public BasicSearch(VisionSubsystem limelight, TurretSubsystem turret) {

        // Set the subsystems
        visionSubsystem = limelight;
        turretSubsystem = turret;

        // Use the limelight and turret subsystems
        addRequirements(visionSubsystem, turretSubsystem);

    }

    @Override
    public void initialize() {

        SmartDashboard.putBoolean("Basic Search Running", true);

        // Set the initial direction to the closest edge
        if (turretSubsystem.getTurretPositionRadians() < 0) {
            direction = -1;
        }

        else if (turretSubsystem.getTurretPositionRadians() >= 0) {
            direction = 1;
        }

    }

    @Override
    public void execute() {

        // Change directions if we reach an edge
        /*if ((turretSubsystem.getTurretPositionRadians() > SuperstructureConstants.turretEdge && movingRight) || 
            (turretSubsystem.getTurretPositionRadians() < -SuperstructureConstants.turretEdge && !movingRight))
            movingRight = !movingRight;*/
        
        // If the turret is not centered on the target
        if (!visionSubsystem.hasValidTarget()) {

            //If turret is at or over an extrema position, invert the movement direction
            if (turretSubsystem.getTurretPositionRadians() >= SuperstructureConstants.rightTurretExtremaRadians ||
                turretSubsystem.getTurretPositionRadians() <= SuperstructureConstants.leftTurretExtremaRadians){

                direction*=-1;
                
            }

            // Set the turret to run at 25% in the correct direction
            turretSubsystem.runTurretPercent(0.25 * direction);


        }
        else if(visionSubsystem.hasValidTarget()) {

            // Finish wrapping and start tracking
            isFinished = true;
            new Tracking(visionSubsystem, turretSubsystem).schedule(true);

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

        SmartDashboard.putBoolean("Basic Search Running", false);

    }

}
