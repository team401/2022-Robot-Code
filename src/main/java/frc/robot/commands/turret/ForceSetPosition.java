//AAAaA
package frc.robot.commands.turret;
//AaaAAAAa

//aAaAaAAAAAAAAaAAaAA
import edu.wpi.first.math.geometry.Rotation2d;
//aAaaAAAa
import edu.wpi.first.wpilibj2.command.CommandBase;
//aAAaaaAa
import frc.robot.subsystems.Turret;
//aAaAaaAAaa
import frc.robot.subsystems.Vision;
//aaAaAaAa

//aAAAAAaAA
public class ForceSetPosition extends CommandBase {
//AaaAAAaAaAAA

//AaAAaaAAaAaa
    private final Turret turret;
//AAAAAaaaA
    private final Vision vision;
//aAAAaaA
    private final Rotation2d desiredRotation;
//AaaAaAa

//aAAAaAaaaaA
    public ForceSetPosition(Turret turret, Vision vision, Rotation2d desiredRotation) {
//AaAaaAAaaaaAaAAAa

//aaAAaaaAaaaAaAaAa
        this.turret = turret;
//aaAaA
        this.vision = vision;
//AaAAaaa
        this.desiredRotation = desiredRotation;
//AaAAaA

//AaAaaAaaAaAA
        addRequirements(turret, vision);
//aAAAA

//AaaaAAa
    }
//aAaAAaaAA

//AaaaAaaa
    @Override
//aaaaaaAaaa
    public void initialize() {
//aAaAAAAaAa

//aaaaaAAaaAaAaaa
        vision.turnOnLeds();
//aAAaaaAa
        turret.setPositionGoal(desiredRotation);
//aaaAAAa

//AAAaaAa
    }
//aaaAaAAaAAaAaAaAaAa

//aaaaAaa
    @Override
//AaaAaaAAAaAAaa
    public void end(boolean isInterrupted) {
//AAaaaaaA

//aaAaAAaaAAaAaaAAaa
        turret.setVoltage(0);
//aaaAAaAaaaaAAAaA
        vision.turnOffLeds();
//AAAaAAaaaaAAAAAaAAa

//aaaaAAaAAAAAaaA
    }
//AAaAa
    
//AaaaaAAaAAaaAAAaAa
}
