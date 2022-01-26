package frc.robot.commands.turret.LimelightBased;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//This command just goes to the closest extrema, then passes it off to wrapping

public class BasicSearch extends CommandBase {

    VisionSubsystem visionSubsystem;
    ShooterSubsystem shooterSubsystem;

    //PID we use to calculate the velocity we should give to our turret and gives in volts
    private PIDController turretSearchingPIDController = new PIDController(0, 0, 0);

    private double direction;
    
    //flag boolean
    private boolean isFinished = false;
    
    public BasicSearch(VisionSubsystem vision, ShooterSubsystem shooter) {
        visionSubsystem = vision;
        shooterSubsystem = shooter;
    }

    @Override
    public void initialize() {
        
        double currentRotation = shooterSubsystem.getTurretPositionRadians();
        if (currentRotation < 0)
            direction = -1;
        else
            direction = 1;

    }

    @Override
    public void execute() {

        if (Math.abs(shooterSubsystem.getTurretPositionRadians()) > Units.degreesToRadians(130)) {

            isFinished = true;
            new Wrapping(visionSubsystem, shooterSubsystem);

        }

        if (!visionSubsystem.isCenteredOnTarget()) {

            double output = turretSearchingPIDController.calculate(shooterSubsystem.getTurretPositionRadians(), direction*Units.degreesToRadians(130));
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
