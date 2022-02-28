package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RotationArmSubsystem;
import frc.robot.subsystems.TelescopeArmSubsystem;
import frc.robot.subsystems.RotationArmSubsystem.Mode;

public class ClimbSequence extends SequentialCommandGroup {

    public ClimbSequence(TelescopeArmSubsystem telescope, RotationArmSubsystem rotation) {

        addCommands(

            // Move rotation arm back to clear the bar
            new UpdateRotationArm(rotation, ClimberConstants.defaultArmPosition, Mode.Climbing),

            // Pull the robot up while holding the rotation arms back
            new UpdateTelescopeArms(telescope, 0)
                .raceWith(new HoldPositionRotationArms(rotation)),
            
            // Rotate the arms to catch the bar while holding the telescope
            new UpdateRotationArm(rotation, 0.1, Mode.Climbing)
                .raceWith(new HoldPositionTelescopeArms(telescope)),
            
            // Push the telescope arms up a little while holding the rotation arms
            new UpdateTelescopeArms(telescope, 5)
                .raceWith(new HoldPositionRotationArms(rotation)),
            
            // Rotate the arms to catch the traversal bar later while holding the telescope
            new UpdateRotationArm(rotation, 0.6, Mode.Climbing)
                .raceWith(new HoldPositionTelescopeArms(telescope)),

            // Extend the telescope arms to catch the traversal bar later while holding the rotation arms
            new UpdateTelescopeArms(telescope, 30)
                .raceWith(new HoldPositionRotationArms(rotation)),

            // Rotate the arms to hit the traversal bar while holding the telescope
            new UpdateRotationArm(rotation, 0.52, Mode.Climbing)
                .raceWith(new HoldPositionTelescopeArms(telescope)),

            // Hook onto the traversal bar and pull up enough to get off the previous bar while holding the rotation arms
            new UpdateTelescopeArms(telescope, 0)
                .raceWith(new HoldPositionRotationArms(rotation)),

            // Move rotation arm back to clear the bar
            new UpdateRotationArm(rotation, ClimberConstants.defaultArmPosition, Mode.Climbing)
                .raceWith(new HoldPositionTelescopeArms(telescope)),
            
            // Pull the robot up while holding the rotation arms back
            new UpdateTelescopeArms(telescope, 0)
                .raceWith(new HoldPositionRotationArms(rotation))
            
            // Hopefully at 3rd bar?
            
            /*
            OLD CODE
            // Retract telescopes and stay there
            new UpdateRotationArm(rotation, ClimberConstants.defaultArmPosition, Mode.Climbing),
            new UpdateTelescopeArms(telescope, 0),
            //new WaitCommand(ClimberConstants.climberSequencePauseSeconds),

            new HoldPositionTelescopeArms(telescope)
            .raceWith(new UpdateRotationArm(rotation, 0.1, Mode.Climbing)),
            // Rotate arms to straight up  
            //new WaitCommand(ClimberConstants.climberSequencePauseSeconds),

            // Extend telescopes to 5 in
            new UpdateTelescopeArms(telescope, 5)
            .raceWith(
                new HoldPositionRotationArms(rotation),
                new WaitCommand(2)),
            //new WaitCommand(ClimberConstants.climberSequencePauseSeconds),

            // Rotate arms to 0.6
            new UpdateRotationArm(rotation, 0.6, Mode.Climbing)
            .raceWith(new HoldPositionTelescopeArms(telescope)),
            //new WaitCommand(ClimberConstants.climberSequencePauseSeconds),

            // Extend telescopes
            new UpdateTelescopeArms(telescope, 30)
            .raceWith(new HoldPositionRotationArms(rotation)).withTimeout(5),
            //new WaitCommand(ClimberConstants.climberSequencePauseSeconds),

            // Rotate arms to 0.52 (unsure about measurement)
            new UpdateRotationArm(rotation, 0.52, Mode.Climbing)
            .raceWith(new HoldPositionTelescopeArms(telescope)),
            //new WaitCommand(ClimberConstants.climberSequencePauseSeconds),

            // Retract telescopes
            new UpdateTelescopeArms(telescope, 0.1)
            .raceWith(new HoldPositionRotationArms(rotation))
            //new HoldPositionTelescopeArms(telescope)
            // Hopefully at 3rd bar?*/
 
        );

    }
    
}
