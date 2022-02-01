package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;

//lil command to run the drive at a given percent in order to test

public class RunAtPercent extends CommandBase {

    private final DriveSubsystem drive;

    //constructor
    public RunAtPercent(DriveSubsystem subsystem) {

        drive = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute() {

        //runs both the drive motors and rotation motors
        drive.runTestatPercentSpin(0.2);
        drive.runTestatPercent(0.2);

    }
    
    @Override
    public void end(boolean in) {

        //stops them at the end
        drive.stopDriving();

    }
    
}
