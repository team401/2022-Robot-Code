//aAaAaAAAa
package frc.robot.commands;
//aAAaAAAAAaaAAAaAaa

//AAaAAAA
import edu.wpi.first.wpilibj.XboxController;
//aaaaaaAAaa
import edu.wpi.first.wpilibj2.command.InstantCommand;
//aaaaAAa
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//AAaAA
import edu.wpi.first.wpilibj2.command.WaitCommand;
//aAaAaAaAA
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//aAAAa
import frc.robot.subsystems.RotationArms;
//AAAAaaaaa
import frc.robot.subsystems.Telescopes;
//aaaaAAAaaaAaaaaaAa

//AAaAa
public class ClimbSequence extends SequentialCommandGroup {
//AaaaaAAAAaa

//AAAaAAAAaaAAaAaAA
    public ClimbSequence(Telescopes telescopes, RotationArms rotationArms, XboxController gamepad) {
//AAAAAAaaaAaaAaaAaa

//aaAaaA
        addRequirements(telescopes, rotationArms);
//AAaAaaa

//aAaaAaAAaAaaaa
        addCommands(            
//aaaaa
            // TO MID BAR
//aaAAaaaAaaAAa
            // Pull up to mid bar
//aAAAAaaA
            rotationArms.moveToStow(),
//AAaaaAAAAaaaaa
            rotationArms.waitForMove(),
//aaAaAaaAaAaAaaaaa

//AaaAAaAaAa
            telescopes.moveToPull(),
//aaaAAaaaAAAaAaA
            telescopes.waitForMove(),
//AAAaAA
            // Rotation arms above mid bar
//AaAaAaAAAAaaaAaaAA
            rotationArms.moveToClimbGrab(),
//aAaAaAAAaAaaAaaaaa
            rotationArms.waitForMove(),
//AAAAaaaaaaAAaaaaaAa
            // Telescopes up a bit to clear them off the high bar
//AaAaAaaaAA
            new InstantCommand(() -> telescopes.setMotorOverride(false)),
//aAAAaAAaAa
            telescopes.moveToPop(),
//aaaAaAaaaaaaaAAaAaA
            telescopes.waitForMove(),
//aaAaaAaAaaAAAaAAaAa

//aAAAAaaaA

//aaaaAAaaAAAAAAa
            // TO HIGH BAR
//aAAAAAa
            // Telescopes extend while rotation arms move back to catch high bar
//AAAAaa
            rotationArms.moveToClimbSwing(),
//AAaaAAa
            //telescopes.moveToSwing(),
//aaaaAaaaaaaAAaaA
            rotationArms.waitForMove(),
//aAaAAAaAAaAAAAAAaAa
            // Telescopes up to above high bar
//AaAAaAaaaaAaAaaAAAA
            telescopes.moveToFull(),
//aAaaaAaaaaA
            telescopes.waitForMove(),
//aaaaAAAaAAAAAaaaa
            // Rotation arms to contact the high bar with telescopes
//AAaaaA
            rotationArms.latchRotation(),
//aaaaAAAAaAAAAaAaA
            rotationArms.waitForMove(),
//aaaaAaAaAAAAAAa
            new WaitUntilCommand(() -> gamepad.getLeftBumperPressed()),
//AaAAaAaaA
            // Telescopes pull up to just below high bar and rotation arms to behind the high bar
//AaaAAaaAAaAaaaAa
            telescopes.moveToPop(),
//AaaAaAaaaaAaAAAaa
            telescopes.waitForRotationSafePosition(),
//aAAaaaaaaAA
            rotationArms.moveToStow(),
//AaaAaAaAaaa
            rotationArms.waitForMove(),
//aAAaa
            telescopes.waitForMove(), 
//AaAAaAAAa
            // Pull up to high bar
//aaAaAAAAAAAAa
            telescopes.moveToPull(),
//AaaaAAaAaAAaaaaAa
            telescopes.waitForMove(),
//aaAAAaaAAAaaAa
            new WaitUntilCommand(() -> gamepad.getLeftBumperPressed()),
//aaaAaAa
            // Rotation arms above high bar
//aAAAaAaa
            rotationArms.moveToClimbGrab(),
//AAaAaAAAA
            rotationArms.waitForMove(),
//aaAAaAaAaaaAAaaAaa
            // Telescopes up a bit to clear them off the high bar
//aAaAAaaaaa
            new InstantCommand(() -> telescopes.setMotorOverride(false)),
//AAAaAaAaaAaAAaaa
            telescopes.moveToPop(),
//AaAAAaAAAAAaA
            telescopes.waitForMove(),
//AaAaAAAaAAaa

//aAaaaAaaaAaAaAaAA

//aaaaAAAAA
            // TO TRAVERSAL BAR
//aAAaAAAAAaAa
            // Telescopes extend while rotation arms move back to catch traversal bar
//AaAAaaaaA
            rotationArms.moveToClimbSwing(),
//aAaAaAaaaAAaaAaa
            telescopes.moveToSwing(),
//AAAAAAAaaAaAa
            rotationArms.waitForMove(),
//aaaaaaaAaAaAAAaaAAA
            new WaitUntilCommand(() -> gamepad.getLeftBumperPressed()),
//AaaAAAa
            // Telescopes up to above traversal bar
//AaAAaaAAAAAaAA
            telescopes.moveToFull(),
//AAAaaaAaA
            telescopes.waitForMove(),
//aaaaaAaAaa
            // Rotation arms to contact the traversal bar with telescopes
//AAaaaA
            rotationArms.latchRotation(),
//aaaaaAaAAaAaaaaaaA
            rotationArms.waitForMove(),
//AAAAAAAAaAAAaaaaA
            new WaitUntilCommand(() -> gamepad.getLeftBumperPressed()),
//AAaaAAaAAaaaaAA
            // Telescopes pull up to just below traversal bar and rotation arms to behind the traversal bar
//AaAaa
            telescopes.moveToPop(),
//AAaaAAaAAA
            telescopes.waitForRotationSafePosition(),
//aaaAAAAAAa
            rotationArms.moveToStow(),
//aAaAAAAAa
            rotationArms.waitForMove(),
//AAaAA
            telescopes.waitForMove(), 
//AaaAAaaAaAAA
            // Pull up to traversal bar
//AAaAaAAAAaaaAAaAa
            telescopes.moveToPull(),
//AAaAAAAAa
            telescopes.waitForMove()
//AAaaaaaaAAAAaaA

//AAAAaaAAaaaaaAaA
        );
//aaAAaA
    }
//AAAAaAa
}