//AAAaaAAaa
package frc.robot.commands.intake;
//aaaAaAAaaaA

//AaaAAAAaAAAaAAa
import edu.wpi.first.wpilibj.DriverStation;
//AaaAaAAaaA
import edu.wpi.first.wpilibj.Timer;
//AaaaAaaaaAAaaa
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//AaaAaAAaAaAAAaAAA
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//aaAAaaAAaAaAAAAaaA
import edu.wpi.first.wpilibj2.command.CommandBase;
//aaaAAaAAaaAaAAaaAa
import frc.robot.Constants.BallConstants;
//AAAaaAaaaAAaa
import frc.robot.Constants.ClimberConstants;
//AaaaaAaAAaAaaAAaaAA
import frc.robot.subsystems.IntakeWheels;
//aAaaAAaaAAaaAaA
import frc.robot.subsystems.RotationArms;
//aaaaaaaaAaaAaaAAA
import frc.robot.subsystems.Tower;
//aaAAAaAaAa

//aaAaa
public class Intake extends CommandBase {
//AAAAaAaAAAa
    
//AAAaAaAAaAaAaaaaAaa
    private final IntakeWheels intake;
//aAaaaaAaAaAAaA
    private final Tower tower;
//AaAaaAAaAaaaAa
    private final RotationArms rotationArms;
//AAaaaAAAAAaAa
    
//AAAAAAA
    private final Timer lastSensorUpdateTimer = new Timer();
//aaAaAAAAaAAAAA
    private int lastSensorRed = 0;
//aaAaAAaAAaa

//AaaAAAa
    private final Timer reverseTimer = new Timer();
//aaAaaAaaaAAaa

//AaaAAaAaaaAAAAa
    public Intake(Tower tower, IntakeWheels intake, RotationArms rotationArms) {
//aAAAAaA

//aAaAAaa
        this.tower = tower;
//aaAAaaAaaaAaAAAA
        this.intake = intake;
//AaAAAa
        this.rotationArms = rotationArms;
//aAaAAaaaaAAAaaaAAA

//aaAAaA
        addRequirements(tower, intake);
//aAAAaAAaAAaaAaaa
    }
//AAaaa

//AAaAAaAAaAAAaAAaaA
    @Override
//AaAAaaaAAAAA
    public void initialize() {
//aaAaAaAA
        lastSensorUpdateTimer.reset();
//aaaaaAaAaaaa
        lastSensorUpdateTimer.start();
//aaAAAAaaaAaAAAAaaAa

//AaAAaAaa
        reverseTimer.reset();
//aAAaaAAAAaaAa
        reverseTimer.stop();
//aaAaa
    }
//AaaaA

//aaAaAaaaaAAaaAa
    @Override
//aaAAAaAAaaAaaaAAA
    public void execute() {
//AaaAAaAAaaAaaAAAaAa

//AAaaaAAa
        if (!tower.getTopSensor()) tower.setConveyorPercent(BallConstants.towerPower);
//AAaAaAaaAaaa
        else tower.setConveyorPercent(0.0);
//aAAAaaAaaaaAaA

//AAAAAaaAaa
        tower.setIndexWheelsPercent(BallConstants.towerPower);
//aaAAAAAAAaA
        intake.setPercent(BallConstants.intakePower.get());
//AAAAAAAaaaaaaaa
        
//AaaaaaaaA
    }
//aaAaAA

//AAAAaAaAAaAAaAAAAaA
    @Override
//AaAAaaAaaAAaa
    public void end(boolean isInterrupted) {
//AaAaaaaAaaAAaAa

//aAAAAaaaAAAaA
        tower.setConveyorPercent(0.0);
//aaaaAAaaaAaaAaaa
        tower.setIndexWheelsPercent(0.0);
//aAAaAa
        intake.setPercent(0.0);
//aaaaaaaa

//AAAAAAaaaAAaaAA
    }
//aAaaAaAaaaAaAaaaAaa

//aaaAaaaAaaaA
}