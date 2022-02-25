package frc.robot.commands.superstructure.ballHandling;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Test extends CommandBase {

    private final IndexingSubsystem indexingSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    
    public Test(IndexingSubsystem index, IntakeSubsystem intake) {

        indexingSubsystem = index;
        intakeSubsystem = intake;
        
        addRequirements(indexingSubsystem, intakeSubsystem);

    }
    
    @Override
    public void initialize() {
        
        SmartDashboard.putBoolean("Test Running", true);

    }

    @Override
    public void execute() {

        if(!indexingSubsystem.getTopBannerState()) indexingSubsystem.runConveyor();
        else indexingSubsystem.stopConveyor();

        indexingSubsystem.runIndexWheels();
        intakeSubsystem.runIntakeMotor();

    }

    @Override
    public void end(boolean isInterrupted) {

        indexingSubsystem.stopConveyor();
        indexingSubsystem.stopIndexWheels();
        intakeSubsystem.stopIntakeMotor();

    }

}