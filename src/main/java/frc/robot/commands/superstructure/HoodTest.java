package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodTest extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;

    public HoodTest(ShooterSubsystem shooter) {

        shooterSubsystem = shooter;

        addRequirements(shooterSubsystem);

    }

    @Override
    public void execute() {

        

    }


    
}
