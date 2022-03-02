package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class Shoot extends CommandBase {

    IndexingSubsystem indexingSubsystem;

    public Shoot(IndexingSubsystem index) {
        indexingSubsystem = index;

        addRequirements(indexingSubsystem);
    }

    @Override
    public void execute() {

        indexingSubsystem.runConveyor();
        indexingSubsystem.runIndexWheels();

    }

    @Override
    public void end(boolean isInterrupted) {

        indexingSubsystem.stopConveyor();
        indexingSubsystem.stopIndexWheels();

    }
    
}
