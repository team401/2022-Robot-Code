package frc.robot.commands.superstructure.ballHandling;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Vomit extends CommandBase {

    private final IndexingSubsystem indexingSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public Vomit(IndexingSubsystem index, IntakeSubsystem intake) {

        indexingSubsystem = index;
        intakeSubsystem = intake;

        addRequirements(index);

    }

    @Override
    public void execute() {

        intakeSubsystem.runIntakeMotor();
        indexingSubsystem.vomitWithIndexWheels();

    }
    
}
