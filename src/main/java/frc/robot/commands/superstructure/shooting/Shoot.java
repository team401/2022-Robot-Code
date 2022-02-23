package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.superstructure.ballHandling.IndexFirstBall;
import frc.robot.commands.superstructure.ballHandling.IndexSecondBall;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
    /**
     * Will shoot ball at speed using pid controller in subsystem
     */

    private final ShooterSubsystem shooterSubsystem;
    private final IndexingSubsystem indexingSubsystem;

    private final Timer timer = new Timer();

    private double ballShotCount = 0;

    private boolean previousTopBannerStatus = false;

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
        indexingSubsystem.runIndexWheels();
        indexingSubsystem.runConveyor();

        // If the top sensor sees a ball, stop
        if (indexingSubsystem.getTopBannerState()) {

            indexingSubsystem.stopIndexWheels();
            indexingSubsystem.stopConveyor();

        }

        // this whole time count how many times the top sensor has lost a ball, then add it to ball count
        if (indexingSubsystem.getTopBannerState() == false && previousTopBannerStatus == true)
        {
            ballShotCount++;
        }

        // Update banner status every 20ms
        previousTopBannerStatus = indexingSubsystem.getTopBannerState();

    }

    @Override
    public boolean isFinished() {

        return timer.get() > 2;

    }

    @Override
    public void end(boolean interrupted) {

        shooterSubsystem.stopShooter();
        shooterSubsystem.stopFeeder();

        if(ballShotCount == 1) {
            
            if (indexingSubsystem.getTopBannerState()) {

                new IndexFirstBall(indexingSubsystem).schedule();
                
            }

            new IndexSecondBall(indexingSubsystem).schedule();

        }
        else if(ballShotCount == 2) {
            
            new IndexFirstBall(indexingSubsystem).schedule();
            
        }

    }

}
