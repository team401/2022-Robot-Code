//AaaAaaAAaAAAAa
package frc.robot.commands.shooter;
//AaaaAaaaaaA

//aAaaaaAaAAaaaAAAaAa
import edu.wpi.first.wpilibj.Timer;
//aaAaaA
import edu.wpi.first.wpilibj2.command.CommandBase;
//aAAAaaaAaaaaaaaA
import frc.robot.subsystems.Shooter;
//aaaAaAAAAAAAaaaa
import frc.robot.subsystems.Tower;
//aaAAAaAAAaAaaA

//aAaAAAaaaaAaaaAaa
public class Shoot extends CommandBase {
//aaAAAaaaa

//AaAaaAaA
    private final Tower tower;
//aaaAAAa
    private final Shooter shooter;
//AaAaaa

//aaaaaa
    private final Timer timer = new Timer();
//aAaaaaAaa

//AaAAaA
    public Shoot(Tower tower, Shooter shooter) {
//aAaaaaAaaaaaAAa

//AAaaaAaA
        this.shooter = shooter;
//AAAaAaaaAaAaAaAa
        this.tower = tower;
//aaaAAAaAaaAaAaaAa

//AaAAaaAaaaaAAaaAAa
        addRequirements(tower);
//AaaaaaaaAaaaAAAaAAA

//AAAAAA
    }
//aaaAAAAaaaAAaAaAaa

//AaaAaaaa
    @Override
//aAAAAaAaaA
    public void initialize() {
//AaAaaAaaAA

//AAAaaAAAAAa
        timer.reset();
//aaaAAaAAA
        timer.start();
//AaaaAAaAaaaA

//AaaAaaAAAAaa
    }
//aaAaAAAAaaAaAaAAAa

//AAAaaAaaAaAa
    @Override
//aaaAaaaaaaaAaaa
    public void execute() {
//AAaAaAAaaAAAaa

//aAAAAA
        if (!shooter.atGoal()) {
//AAAaaaaAAaa
            timer.reset();
//AAAAaAAaAaAaAAaaaaA
            tower.setConveyorPercent(0.0);
//aaAAa
            tower.setIndexWheelsPercent(0.0);
//AAAaaAa
        }
//aaaAAaAAA

//AaaaAa
        if (timer.get() > 0.1) {
//aAAAA
            tower.setConveyorPercent(1.0);
//AaaaAaaaAaAaAAAAaa
            tower.setIndexWheelsPercent(1.0);
//aAAaaaa
        }
//aaaaAaAaAAaAaaAa

//aAAAaAaAAaaAAA
    }
//aaAaaAAaaaAAa

//aaaAaAaAAAAAaA
    @Override
//AaaAaaaaa
    public void end(boolean isInterrupted) {
//AaAaaAAaAaa
        tower.setConveyorPercent(0.0);
//aAAAAaaa
        tower.setIndexWheelsPercent(0.0);
//AAaaAAA
    }
//AaaaAAAaA
    
//AAAaaaaaAAAaaaAAAAa
}
