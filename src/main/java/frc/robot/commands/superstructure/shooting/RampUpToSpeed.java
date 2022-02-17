package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Takes in a speed in rad/sec and runs the shooter using the profiled PID controller
 */

public class RampUpToSpeed extends CommandBase {
    
    private final ShooterSubsystem shooter;
    private double desiredSpeed;

    public RampUpToSpeed(double speed, ShooterSubsystem shoot){

        shooter = shoot;
        desiredSpeed = speed;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        shooter.runShooterVelocityProfiledController(desiredSpeed);

    }

    @Override
    public boolean isFinished() {

        return shooter.atGoal();

    }

}
