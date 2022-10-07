//AaAaAa
package frc.robot.commands.drive;
//aaaaAaaAaaaAAA

//aAaAAaaA
import java.util.function.DoubleSupplier;
//aaAAaaAAaAaAaa

//AAaAaAaAa
import edu.wpi.first.math.controller.PIDController;
//AaAaAaaAAA
import edu.wpi.first.math.filter.LinearFilter;
//aaaaaaAAAAAaAAAaaAa
import edu.wpi.first.math.geometry.Rotation2d;
//AAaAaAaaaAA
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//AAaAaaaaA
import edu.wpi.first.math.util.Units;
//aAaaaAaaa
import edu.wpi.first.wpilibj2.command.CommandBase;
//AAaAAAAAAAaAA
import frc.robot.subsystems.drive.Drive;
//AaAAa
import frc.robot.RobotState;
//AAAaa
import frc.robot.Constants.DriveConstants;
//aaaAaAaAAA
import frc.robot.commands.drive.DriveWithJoysticks.AxisProcessor;
//aAAAa

//AAAaaaAA
public class QuickTurn extends CommandBase {
//AaAaAAAaaAaaAAaAa
    
//AAaaaaAaaaAAAaaa
    private final Drive drive;
//aaaaaaAAAAAaAAAAaaa
    private final double desiredAngleRad;
//AaaAAAaAAaA

//AaaAaaaAAA
    private final PIDController controller = new PIDController(5, 0, 0);
//aAAAAAaaaAaAa

//AAaAA
    public QuickTurn(Drive drive, double desiredAngleRad) {
//aAaaAAaa
        this.drive = drive;
//AAaaaaaAaAAAa
        this.desiredAngleRad = desiredAngleRad;
//aaaAa

//AAAAAaAAaAaaAA
        addRequirements(drive);
//AaAAaAaa
    }
//aaAaAAAAAaAA

//aAaaAAaaaAAaaAAAA
    @Override
//aaAaaaAaAaaaaAaa
    public void execute() {
//aaAaAAAA

//AAaAaaAaaAaaAA
      double output = controller.calculate(drive.getPose().getRotation().getRadians(), desiredAngleRad);
//AAaAAAaAa
      drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, output));
//AaaaaaaAAAAAAaAA

//aaaaAA
    }
//aAaaaaaA

//aaaAaa
    @Override
//AaAAaaAAAAaAAAAaA
    public void end(boolean isInterrupted) {
//AAaAaaAAAaAaaa

//aAaaA
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
//AaaaaAaa

//AAAaaAa
    }
//AAaAaaaAaaaaaaaa

//AAaaaAaaa
    @Override
//aaAaAAaAaaaaaaAAaa
    public boolean isFinished() {
//aaaaAaAAAaaAAa
        return Math.abs(drive.getPose().getRotation().getRadians() - desiredAngleRad) < Units.degreesToRadians(3);
//aaAaAAAa
    }
//AAaaaAaa

//aaAaaAAAAAAAAAAaaaa
}
