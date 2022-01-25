package frc.robot.commands.turret.LimelightBased;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**TO DO
 * Wrapping basically makes the turret go from the closest end to the farthest and oscillates between the
 * two points until it finds the target
 * Once it finds target, it goes to searching
 */

public class Wrapping extends CommandBase {
    
    VisionSubsystem visionSubsystem;
    ShooterSubsystem shooterSubsystem;
    
    //constructor
    public Wrapping(VisionSubsystem vision, ShooterSubsystem shooter) {

        visionSubsystem = vision;
        shooterSubsystem = shooter;

        addRequirements(vision, shooter);

    }

    @Override
    public void execute() {

    }
    

}
