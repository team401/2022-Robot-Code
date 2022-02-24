package frc.robot.commands.superstructure.ballHandling;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

/**
 * Wait until bottom banner sensor is tripped, then go to feeding
 */

public class Waiting extends CommandBase {

    private final IndexingSubsystem indexingSubsystem;
    
    public Waiting(IndexingSubsystem index) {

        indexingSubsystem = index;

        addRequirements(indexingSubsystem);

    }

    @Override
    public void execute() {

        if (indexingSubsystem.getBottomBannerState()) {

            new Feeding(indexingSubsystem).schedule(true);

        }

    }

    @Override
    public boolean isFinished() {

        return indexingSubsystem.getBottomBannerState();

    }

}