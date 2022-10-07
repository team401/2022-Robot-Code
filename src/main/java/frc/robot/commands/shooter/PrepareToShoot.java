//aaaAAAaaaaAAAAa
package frc.robot.commands.shooter;
//AaAAAaaaAAaaaaAaaA

//aaAAaA
import edu.wpi.first.math.util.Units;
//aaAaaAAaaAaAaaAAAAa
import edu.wpi.first.wpilibj.DriverStation;
//aAAAaaaaAaAaaaaAaa
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//AaAAaAAA
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//AAAAAaAaa
import edu.wpi.first.wpilibj2.command.CommandBase;
//AAaAAAAAAA
import frc.robot.Constants;
//aAAaaa
import frc.robot.RobotState;
//AAAAaAAaaaAa
import frc.robot.Constants.ShooterConstants;
//AAaAaAAAAAAAAaaaA
import frc.robot.subsystems.Shooter;
//AaaaAAAAAAAaaaAAAaa
import frc.robot.subsystems.Tower;
//AaAaaaAaaAaaAAaA
import frc.robot.util.Interpolation.InterpolatingDouble;
//AAaAAaA

//AaAaAAaaaAAAAA
public class PrepareToShoot extends CommandBase {
//AaaaaAAaaaaAAAaAa

//aaAaaaa
    private final Shooter shooter;
//aaAAaaaAAAAAA
    private final Tower tower;
//AaAaAaAAaA

//AaaAAAaA
    public PrepareToShoot(Shooter shooter, Tower tower) {
//AAaAaAa

//AAaaAaAaaaaAAAaaAAA
        this.shooter = shooter;
//AaAaAAaAAaaAaA
        this.tower = tower;
//AAAAAAaAaAAAaaaAA

//aAAaaaaAAaAAA
        addRequirements(shooter);
//aAAAaaaAA

//AAAaaAaaAaA
    }
//AAaAaAaaAAaaa

//AAAAAAaAAAaAAa
    @Override
//AaaAAaaAAAAaaaAaa
    public void execute() {
//AAaAaaA

//aAaaaAaaAa
        RobotState.AimingParameters params = RobotState.getInstance().getAimingParameters();
//AaaAaAa
        double hoodAngle = Constants.ShooterConstants.hoodLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value;
//AAaAaaAAAA
        double shotSpeed = Units.rotationsPerMinuteToRadiansPerSecond(Constants.ShooterConstants.flywheelLookup.getInterpolated(new InterpolatingDouble(params.getDistanceM())).value);
//AaaaaaAA

//aaaAaAAAAAAAaaaAaA
        //if (RobotState.getInstance().hasCorrectBall())
//AaaaaAaAaaAaaAAaaA
        shooter.setSetpoint(hoodAngle, shotSpeed);
//AAAaAaaAAaAAA
        //else
//aaAaAA
            //shooter.setSetpoint(hoodAngle, Units.rotationsPerMinuteToRadiansPerSecond(Constants.ShooterConstants.intentionalMissRPM));
//AAAaAaaA
        
//aAaaAAAaAAAAaA
        //shooter.setSetpoint(SmartDashboard.getNumber("Hood Desired", 0.27), Units.rotationsPerMinuteToRadiansPerSecond(SmartDashboard.getNumber("Shooter Desired", 0)));
//AaAAAAAaaAAaA
        
//AAAaaaAaAaAAA
    }
//aaAAAaaAaaaAaAaAa

//AaaaaAAAAAa
    @Override
//aAaAAaAAa
    public void end(boolean interrupted) {
//AaaaaAAAAaa
        shooter.stopShooter();
//aaAaaaAAaaaaaA
    }
//aAaAAAaaAaaaAaaAa
}