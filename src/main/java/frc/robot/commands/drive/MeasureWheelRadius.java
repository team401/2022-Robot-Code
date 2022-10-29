package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class MeasureWheelRadius extends CommandBase {

    /**
     * Command to measure the wheel radius of the drive
     * For swerve, we assume each module wheel radius to be the same and use the average distance driven as the robot distance
     */

    private final Drive drive;

	private double distance;

	private double startRad;

    public MeasureWheelRadius(double distanceFt, Drive subsystem) {

        drive = subsystem;

        distance = distanceFt;

    }

    @Override
    public void initialize() {

        startRad = drive.getModuleDistances();

    }

	@Override
	public void execute() {
		drive.setGoalModuleStates(new SwerveModuleState[] {
			new SwerveModuleState(0.0, new Rotation2d(0)),
			new SwerveModuleState(0.0, new Rotation2d(0)),
			new SwerveModuleState(0.0, new Rotation2d(0)),
			new SwerveModuleState(0.0, new Rotation2d(0))
		});
	}

    @Override
    public void end(boolean interrupted) {

        double distanceRadians = drive.getModuleDistances() - startRad;

        SmartDashboard.putNumber("wheel radius in", 12 * distance / distanceRadians);

    }

}