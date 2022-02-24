package frc.robot.commands.superstructure.turret.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//maybe inline?

public class Locked extends CommandBase {

    private final VisionSubsystem visionSubsystem;
    private final TurretSubsystem turretSubsystem;

    public Locked(VisionSubsystem limelight, TurretSubsystem turret) {

        visionSubsystem = limelight;
        turretSubsystem = turret;

        addRequirements(visionSubsystem, turretSubsystem);

    }

    @Override
    public void execute() {

        turretSubsystem.runTurretPercent(0);

    }
    
}
