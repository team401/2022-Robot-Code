package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.superstructure.ballHandling.Waiting;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
    
    /**
     * Will shoot ball at speed using pid controller in subsystem
     */

    private final ShooterSubsystem shooterSubsystem;
    private final IndexingSubsystem indexingSubsystem;

    private final Timer timer = new Timer();

    public Shoot(ShooterSubsystem shoot, IndexingSubsystem index){

        shooterSubsystem = shoot;
        indexingSubsystem = index;

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        if (shooterSubsystem.atGoal()) {

            shooterSubsystem.runFeederPercent(0.75);

        } else {

            shooterSubsystem.stopFeeder();
            timer.reset();

        }

        // RUN FOREST RUN
        indexingSubsystem.runConveyor();
        indexingSubsystem.runIndexWheels();

    }

    @Override
    public boolean isFinished() {

        return timer.get() > 2;

    }

    @Override
    public void end(boolean interrupted) {

        indexingSubsystem.stopConveyor();
        indexingSubsystem.stopIndexWheels();

        new Waiting(indexingSubsystem).schedule();

    }

}
