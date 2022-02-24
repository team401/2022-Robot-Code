package frc.robot.commands.superstructure.ballHandling;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

/**
 * Runs the conveyor and index motors until top banner is tripped
 */

public class Feeding extends CommandBase {

    private final IndexingSubsystem indexingSubsystem;
    
    public Feeding(IndexingSubsystem index) {

        indexingSubsystem = index;

        addRequirements(indexingSubsystem);

    }

    @Override
    public void execute() {

        indexingSubsystem.runConveyor();
        indexingSubsystem.runIndexWheels();

        if (indexingSubsystem.getTopBannerState()) {
            indexingSubsystem.stopConveyor();
            indexingSubsystem.stopIndexWheels();

            new Waiting(indexingSubsystem).schedule(true);
        }

    }

    @Override
    public boolean isFinished() {

        return indexingSubsystem.getTopBannerState();

    }

}