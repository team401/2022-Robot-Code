package frc.robot.commands.turret.LimelightBased;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**TO DO
 * Wrapping basically makes the turret go from the closest end to the farthest and oscillates between the
 * two points until it finds the target
 * Once it finds target, it goes to searching
 */

public class Wrapping extends CommandBase {
    
    VisionSubsystem visionSubsystem;
    ShooterSubsystem shooterSubsystem;

    //PID we use to calculate the velocity we should give to our turret and gives in volts
    private PIDController turretWrappingPIDController = new PIDController(0, 0, 0);

    private double direction;

    //flag boolean
    private boolean isFinished = false;
    
    //constructor
    public Wrapping(VisionSubsystem vision, ShooterSubsystem shooter) {

        visionSubsystem = vision;
        shooterSubsystem = shooter;

        addRequirements(vision, shooter);

    }

    @Override
    public void initialize() {

        double currentRotation = shooterSubsystem.getTurretPositionRadians();
        if (currentRotation < 0)
            direction = 1;
        else
            direction = -1;

    }

    @Override
    public void execute() {

        if (Math.abs(shooterSubsystem.getTurretPositionRadians()) > Units.degreesToRadians(130)) 
            direction = -direction;

        if (!visionSubsystem.isCenteredOnTarget()) {

            double output = turretWrappingPIDController.calculate(shooterSubsystem.getTurretPositionRadians(), direction*Units.degreesToRadians(130));
            shooterSubsystem.runShooterPercent(output);

        }
        else {

            isFinished = true;
            new Tracking(visionSubsystem, shooterSubsystem);

        }

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
    
}
