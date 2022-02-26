package frc.robot.commands.superstructure.turret.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

//maybe inline?

public class Locked extends CommandBase {

    private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;

    public Locked(LimelightSubsystem limelight, TurretSubsystem turret) {

        limelightSubsystem = limelight;
        turretSubsystem = turret;

        addRequirements(limelightSubsystem, turretSubsystem);

    }

    @Override
    public void execute() {

        turretSubsystem.runTurretPercent(0);

    }
    
}
