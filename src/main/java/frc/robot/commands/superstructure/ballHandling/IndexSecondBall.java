package frc.robot.commands.superstructure.ballHandling;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class IndexSecondBall extends CommandBase {

    private final IndexingSubsystem indexingSubsystem;

    private boolean tripped = false;
    
    public IndexSecondBall(IndexingSubsystem index) {

        indexingSubsystem = index;

        addRequirements(indexingSubsystem);

    }

    @Override
    public void execute() {

        if(indexingSubsystem.getBottomBannerState()) {

            tripped = true;

        }
        if(tripped) {

            indexingSubsystem.runIndexWheels();
            indexingSubsystem.runConveyor();

            if(indexingSubsystem.getTopBannerState()) {

                indexingSubsystem.stopIndexWheels();
                indexingSubsystem.stopConveyor();

            }

        }

    }

    @Override
    public boolean isFinished() {

        return tripped && indexingSubsystem.getTopBannerState();

    }

}