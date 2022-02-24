package frc.robot.commands.superstructure.ballHandling;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class ReverseIndexing extends CommandBase {

    private final IndexingSubsystem indexingSubsystem;

    public ReverseIndexing(IndexingSubsystem index) {
        
        indexingSubsystem = index;
        
    }
    
    @Override
    public void execute() {

        indexingSubsystem.reverseConveyor();
        indexingSubsystem.reverseIndexWheels();

    }

    @Override
    public void end(boolean isInterrupted) {

        indexingSubsystem.stopConveyor();
        indexingSubsystem.stopIndexWheels();

    }
    
}
