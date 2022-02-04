package frc.robot.commands.turret.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * TODO: the execute logic?
 * Goes to the closest extrema, and then goes to wrapping
 */

public class BasicSearch extends CommandBase {

    VisionSubsystem visionSubsystem;
    ShooterSubsystem shooterSubsystem;

    //PID we use to calculate the velocity we should give to our turret and gives in volts
    private PIDController turretSearchingPIDController = new PIDController(0, 0, 0);

    //what direction we are going to use (-1 is to the left, 1 is to the right)
    private double direction;
    
    //flag boolean
    private boolean isFinished = false;
    
    //constructor
    public BasicSearch(VisionSubsystem vision, ShooterSubsystem shooter) {
        
        //initializing subsystmes
        visionSubsystem = vision;
        shooterSubsystem = shooter;

        //what we need to use for the subsystem
        addRequirements(vision, shooter);

    }

    @Override
    public void initialize() {
        
        //find the closest side based on the position
        double currentRotation = shooterSubsystem.getTurretPositionRadians();
        if (currentRotation < 0)
            direction = -1;
        else
            direction = 1;

    }

    @Override
    public void execute() {

        //if statement for 
        if (Math.abs(shooterSubsystem.getTurretPositionRadians()) > SuperstructureConstants.turretEdge) {

            isFinished = true;
            new Wrapping(visionSubsystem, shooterSubsystem);

        } else if (!visionSubsystem.withinTolerance()) {

            double output = turretSearchingPIDController.calculate(
                shooterSubsystem.getTurretPositionRadians(), 
                direction*Units.degreesToRadians(130));
            shooterSubsystem.runShooterPercent(output);

        } else {

            isFinished = true;
            new Tracking(visionSubsystem, shooterSubsystem);

        }

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}