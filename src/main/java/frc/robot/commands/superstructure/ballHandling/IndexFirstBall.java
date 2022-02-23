package frc.robot.commands.superstructure.ballHandling;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

/**
 * Command runs the indexing wheels until ball has passed the lower banner sensor
 */

public class IndexFirstBall extends CommandBase {

    private final IndexingSubsystem indexingSubsystem;
    
    private boolean tripped = false;

    public IndexFirstBall(IndexingSubsystem index) {

        indexingSubsystem = index;

        addRequirements(indexingSubsystem);

    }

    @Override
    public void execute() {

        // If ball hasn't tripped the bottom banner
        if(!tripped && !indexingSubsystem.getBottomBannerState()) {

            indexingSubsystem.runIndexWheels();
            indexingSubsystem.runConveyor();

        }
        // If the ball has tripped the bottom banner for the first time
        else if (!tripped && indexingSubsystem.getBottomBannerState()) {
            
            tripped = true;
            indexingSubsystem.runIndexWheels();
            indexingSubsystem.runConveyor();
            
        }
        // If the ball has passed the bottom banner
        else if (tripped && !indexingSubsystem.getBottomBannerState()) {

            tripped = false;
            indexingSubsystem.stopIndexWheels();
            indexingSubsystem.stopConveyor();

            new IndexSecondBall(indexingSubsystem).schedule();

        }

    }

    @Override
    public boolean isFinished() {

        return tripped && !indexingSubsystem.getBottomBannerState();

    }
    
}
