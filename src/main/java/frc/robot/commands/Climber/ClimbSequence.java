package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RotationArmSubsystem;
import frc.robot.subsystems.TelescopeArmSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.RotationArmSubsystem.Mode;

public class ClimbSequence extends SequentialCommandGroup {

    /**
     * 1: Driver manually extends telescope and lines up bot with 2nd bar
     * 2: Driver presses button and activates climb sequence
     * 3: Retract Telescope fully (0.5)
     * 4: Rotate arms to straight up (0)
     * 5: Extend Telescope 5 in
     * 6: Rotate arms to 0.6
     * 7: Extend Telescope fully
     * 8: Rotate arms to 0.52 (unsure about measurement)
     * 9: Retract Telescope fully
     * 
     */

    public ClimbSequence(TelescopeArmSubsystem telescope, RotationArmSubsystem rotation) {

        addCommands(
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
            // Hopefully at 3rd bar?
 
        );

    }
    
}
