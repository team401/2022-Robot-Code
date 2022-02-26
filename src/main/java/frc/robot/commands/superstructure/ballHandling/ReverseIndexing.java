package frc.robot.commands.superstructure.ballHandling;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIndexing extends CommandBase {

    private final IndexingSubsystem indexingSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ReverseIndexing(IndexingSubsystem index, IntakeSubsystem intake) {
        
        indexingSubsystem = index;
        intakeSubsystem = intake;
        
    }
    
    @Override
    public void execute() {

        indexingSubsystem.reverseConveyor();
        indexingSubsystem.reverseIndexWheels();
        intakeSubsystem.reverseIntakeMotor();

    }

    @Override
    public void end(boolean isInterrupted) {

        indexingSubsystem.stopConveyor();
        indexingSubsystem.stopIndexWheels();
        intakeSubsystem.stopIntakeMotor();

    }
    
}
