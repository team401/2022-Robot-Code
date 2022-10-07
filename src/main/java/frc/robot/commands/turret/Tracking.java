//aaAaaAAAAAAA
package frc.robot.commands.turret;
//aAAaAAA

//aAaaaaaA
import edu.wpi.first.wpilibj2.command.CommandBase;
//aaaaAaaaaAAaaaaAaaa
import edu.wpi.first.math.filter.LinearFilter;
//AAaaaAAAAA
import edu.wpi.first.math.geometry.*;
//aAaAAa

//AaaaaaaaAaAaaaAA
import frc.robot.subsystems.Vision;
//AaaAaaAAaAaAaaA
import frc.robot.RobotState;
//aaaaaAAAAaa
import frc.robot.RobotState.AimingParameters;
//AAAaa
import frc.robot.subsystems.Turret;
//aAAAAAAAA

//AAAaaaaaa
public class Tracking extends CommandBase {
//AAAaaaaaAAa
    
//aaAaAaaaaaAaA
    private final Vision vision;
//aAaAAAAAaAaAAa
    private final Turret turret;
//aAaaaaaaA

//aAAaaaAaaaaAaAa
    private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
//AAAaaAaaAAaaA

//aaAAAAAAaAaA
    public Tracking(Vision vision, Turret turret) {
//aaAAAAAAaAAaa
        this.vision = vision;
//AaAAAaAaaAaaAaAAAAA
        this.turret = turret;
//AaaAAAAaaAAAAaAA

//aaaaAaAa
        addRequirements(vision, turret);
//aaaAaAaa
    }
//aaAAAAAAaaAaaAAA

//AaAaaaaaAAaAaa
    @Override
//AaAaa
    public void initialize() {
//aaAaAaAa
        filter.reset();
//aAaaAA
        vision.turnOnLeds();
//aaaAA
    }
//AAaAAaAaAAAa

//AAAaaAA
    @Override
//AAaAaAAaaaaAAAaAaa
    public void execute() {
//aaaAaaAaaaAAAa

//aAaAaAAAaAAAAaaAaa
        AimingParameters params = RobotState.getInstance().getAimingParameters();
//aAaaAAAAAAaAaaaaA
        double filteredAngle = filter.calculate(params.getTurretAngle().getRadians());
//AAaAAAaaaaAaaAaAa
        turret.setPositionGoal(new Rotation2d(filteredAngle), params.getVelocityRadPerSec());
//aAaaAaaaaAAAAA
        
//aAaAaaaAa
    }
//aaaaAAaAaAAAAAaAA

//AaAAaa
    @Override
//aAaAaaaa
    public boolean isFinished() {
//AAAaAaaAa
        return false;
//AAaaaA
    }
//AAAAaaaaa

//aaaAAAaaaaAAaaaa
    @Override
//AAAaaAAaAaAAA
    public void end(boolean isInterrupted) {
//AAaaaaaaaaAAA
        vision.turnOffLeds();;
//aaAaaAAAaaAAaaAa
    }
//aAaAAaaAaAAaaAA
}
