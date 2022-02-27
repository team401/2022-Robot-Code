package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeArmSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class UpdateTelescopeArms extends CommandBase {

    private final TelescopeArmSubsystem telescope;
    private final TurretSubsystem turretSubsystem; // Turret subsystem is just required to interrupt tracking command
    private double position;

    public UpdateTelescopeArms(TelescopeArmSubsystem climb, TurretSubsystem turret, double desiredPosition) {
        telescope = climb;
        turretSubsystem = turret;
        position = desiredPosition;

        addRequirements(telescope, turretSubsystem);
    }

    @Override
    public void initialize() {
        telescope.resetControllers();
    }

    @Override
    public void execute() {

        telescope.setLeftDesiredPosition(position);
        telescope.setRightDesiredPosition(position);

    }

    @Override
    public void end(boolean interrupted) {

        telescope.setLeftPercent(0);
        telescope.setRightPercent(0);

    }

    @Override
    public boolean isFinished() {
        return telescope.atGoal() || !telescope.withinBoundaries();
    }
    
}
 