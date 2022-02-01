package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;

//lil command to run the drive at a percent in order to test

public class RunAtPercent extends CommandBase {

    private final DriveSubsystem drive;

    public RunAtPercent(DriveSubsystem subsystem) {

        drive = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute() {

        drive.runTestatPercentSpin();
        drive.runTestatPercent();

    }
    
    @Override
    public void end(boolean in) {

        drive.stopDriving();

    }
    
}
