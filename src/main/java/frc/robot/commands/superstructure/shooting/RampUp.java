package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RampUp extends CommandBase {
    
    private final ShooterSubsystem shooter;

    private double desiredSpeed;

    public RampUp(ShooterSubsystem shoot, double desiredSpeedRPM) {

        shooter = shoot;

        desiredSpeed = desiredSpeedRPM;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        shooter.runShooterVelocityController(desiredSpeed);

    }

    @Override
    public boolean isFinished() {

        return shooter.atGoal();

    }
    
}