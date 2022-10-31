package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Tower;

public class ReverseIntake extends CommandBase {

	private final Tower tower;
	private final IntakeWheels intakeWheels;

	public ReverseIntake(Tower tower, IntakeWheels intakeWheels) {
		this.tower = tower;
		this.intakeWheels = intakeWheels;

		addRequirements(tower, intakeWheels);
	}

	@Override
	public void initialize() {
		intakeWheels.setPercent(-0.5);
		tower.setConveyorPercent(-0.5);
		tower.setIndexWheelsPercent(-0.5);
	}

	@Override
	public void end(boolean isInterrupted) {
		intakeWheels.setPercent(0);
		tower.setConveyorPercent(0);
		tower.setIndexWheelsPercent(0);
	}
	
}
