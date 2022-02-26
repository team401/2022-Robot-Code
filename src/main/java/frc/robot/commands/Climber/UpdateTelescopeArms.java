package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class UpdateTelescopeArms extends CommandBase {

    private final ClimbSubsystem climbingSubsystem;
    private final TurretSubsystem turretSubsystem; // Turret subsystem is just required to interrupt tracking command
    private double position;

    public UpdateTelescopeArms(ClimbSubsystem climber, TurretSubsystem turret, double desiredPosition) {
        climbingSubsystem = climber;
        turretSubsystem = turret;
        position = desiredPosition;

        addRequirements(climbingSubsystem, turretSubsystem);
    }

    @Override
    public void initialize() {
        climbingSubsystem.resetTelescopeControllers();
    }

    @Override
    public void execute() {

        climbingSubsystem.setLeftDesiredTelescopePosition(position);
        climbingSubsystem.setRightDesiredTelescopePosition(position);

        SmartDashboard.putNumber("left telescope", position);
        SmartDashboard.putNumber("right telescope", position);

        //climbingSubsystem.setLeftTelescopePercent(0.5);
        //climbingSubsystem.setRightTelescopePercent(0.5);

    }

    @Override
    public void end(boolean interrupted) {

        climbingSubsystem.setLeftTelescopePercent(0);
        climbingSubsystem.setRightTelescopePercent(0);

    }

    @Override
    public boolean isFinished() {
        // TODO: change velocity values

        return climbingSubsystem.atGoalTelescope() || !climbingSubsystem.withinBoundariesTelescope();
    }
    
}
 