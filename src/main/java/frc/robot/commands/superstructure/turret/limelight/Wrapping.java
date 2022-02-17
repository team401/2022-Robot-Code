package frc.robot.commands.superstructure.turret.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**TO DO
 * Wrapping basically makes the turret go from the closest end to the farthest and oscillates between the
 * two points until it finds the target
 * Once it finds target, it goes to searching
 */

public class Wrapping extends CommandBase {
    
    //the instances of the two subsystems needed
    VisionSubsystem visionSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

    //PID we use to calculate the velocity we should give to our turret and gives in volts
    private PIDController turretWrappingPIDController = new PIDController(0, 0, 0);

    //what direction we want to go in (negative is to the left)
    private double direction;

    //flag boolean
    private boolean isFinished = false;
    
    //constructor
    public Wrapping(VisionSubsystem vision, ShooterSubsystem shooter, TurretSubsystem turret) {

        visionSubsystem = vision;
        shooterSubsystem = shooter;
        turretSubsystem = turret;

        //we use the vision subsystem and shooter subsystem for this method
        addRequirements(vision, shooter);

    }

    @Override
    public void initialize() {

        //get the position and see which side is closest, and that determines the direction
        double currentRotation = turretSubsystem.getTurretPositionRadians();
        if (currentRotation < 0)
            direction = 1;
        else
            direction = -1;

    }

    @Override
    public void execute() {

        //if we reach the edge of the turret, invert the direction
        if (Math.abs(turretSubsystem.getTurretPositionRadians()) > SuperstructureConstants.turretEdge) 
            direction = -direction;

        //if the tx is greater than our tolerance (not locked on), we use PID control to send it to the edge 
        if (!visionSubsystem.withinTolerance()) {

            double output = turretWrappingPIDController.calculate(
                turretSubsystem.getTurretPositionRadians(), 
                direction*SuperstructureConstants.turretEdge
            );
            turretSubsystem.runTurretVoltage(output);

        } else { //if we are within the tolerance, we mark our flag as finished and go to tracking

            isFinished = true;
            new Tracking(visionSubsystem, shooterSubsystem, turretSubsystem).schedule();;

        }

    }

    @Override
    public boolean isFinished() {

        return isFinished;

    }
    
}
