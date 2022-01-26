package frc.robot.commands.turret.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**TO DO
 * Searching is a command for the turret which uses the limelight and drive subsytem to spin the turret
 * in the opposite direction as the movement of the drivetrain to track
 * If gets the target, it goes to tracking
 * If it reaches the end or the robot is not moving, it goes to wrapping
 */

public class Searching extends CommandBase {

    VisionSubsystem visionSubsystem;
    ShooterSubsystem shooterSubsystem;
    
    public Searching(VisionSubsystem vision, ShooterSubsystem shooter) {
        visionSubsystem = vision;
        shooterSubsystem = shooter;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean in) {

    }
}
