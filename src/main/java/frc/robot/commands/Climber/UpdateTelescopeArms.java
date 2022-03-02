package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeArmSubsystem;

public class UpdateTelescopeArms extends CommandBase {

    private final TelescopeArmSubsystem telescope;
    private double position;

    public UpdateTelescopeArms(TelescopeArmSubsystem climb, double desiredPosition) {
        telescope = climb;
        position = desiredPosition;

        addRequirements(telescope);
    }

    @Override
    public void initialize() {
        telescope.resetControllers();
    }

    @Override
    public void execute() {

        telescope.setLeftDesiredPosition(position);
        telescope.setRightDesiredPosition(position);

    }

    @Override
    public void end(boolean interrupted) {

        telescope.setLeftPercent(0);
        telescope.setRightPercent(0);

    }

    @Override
    public boolean isFinished() {
        return telescope.atGoal() || !telescope.withinBoundaries();
    }
    
}
 