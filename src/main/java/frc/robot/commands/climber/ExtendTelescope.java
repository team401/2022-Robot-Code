package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ExtendTelescope extends CommandBase {

    private final ClimbSubsystem climbingSubsystem;

    public ExtendTelescope(ClimbSubsystem climber) {
        climbingSubsystem = climber;

        addRequirements(climbingSubsystem);
    }

    @Override
    public void execute() {

        

    }

    @Override
    public void end(boolean isInterrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
 