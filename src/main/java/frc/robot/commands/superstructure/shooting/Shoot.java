package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
    /**
     * Will shoot ball at speed using pid controller in subsystem
     */

    private final ShooterSubsystem shooter;

    private final Timer timer = new Timer();

    public Shoot(ShooterSubsystem shoot){

        shooter = shoot;

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        if (shooter.atGoal()) {

            shooter.runFeederPercent(0.5);

        } else {

            shooter.stopFeeder();
            timer.reset();

        }

    }

    @Override
    public boolean isFinished() {

        return timer.get() > 2;

    }

    @Override
    public void end(boolean interrupted) {

        shooter.stopShooter();
        shooter.stopFeeder();

    }

}
