//aAAAaAAAAAa
package frc.robot.commands.drive;
//aAaAa

//AaaaA
import java.util.function.DoubleSupplier;
//AAaaaaaaaaaAAaaaAaa

//AAaAAa
import edu.wpi.first.math.controller.PIDController;
//AaaAAAAAaAaaAaaAA
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//AAaaAaAAaAAaA
import edu.wpi.first.wpilibj2.command.CommandBase;
//AaAAAa
import frc.robot.Constants.DriveConstants;
//aAAaaAaAaaAaaaAAAaa
import frc.robot.commands.intake.Intake;
//AAaaaAa
import frc.robot.commands.drive.DriveWithJoysticks.AxisProcessor;
//aaAAAAAaAAAAA
import frc.robot.subsystems.drive.Drive;
//AAaaa
import frc.robot.subsystems.IntakeVision;
//aAAAAaaAaaaAAA

//AaAAAaAaaAAAA
public class VisionIntake extends CommandBase {
//AAAaaaaa

//aAAAAaAaAaaaaAAAaaa
    private final IntakeVision intakeVision;
//aAaAaAAA
    private final Drive drive;
//AAaAAaAAAAaaAaaAAa

//aAAaaaaAAa
    private final DoubleSupplier yPercent;
//AAaAaaa
    private final AxisProcessor yProcessor = new AxisProcessor(false);
//AaaaaAaAA

//aaAAaaa
    private final PIDController controller = new PIDController(DriveConstants.intakeVisionKP.get(), 0, DriveConstants.intakeVisionKD.get());
//aAaaAaaa

//aAAaAaaaAaAA
    
//AaAAAAaa
    public VisionIntake(IntakeVision intakeVision, Drive drive, DoubleSupplier yPercent) {
//aAAaAaaaaAAa

//AAAAaaAAaAaaa
        this.intakeVision = intakeVision;
//aaAAAAAAAaaAaAAAA
        this.drive = drive;
//aAAaAAaAaAA

//AAAAAaAAaAaAaAAAa
        this.yPercent = yPercent;
//aAaaaaAAAaaAaAA

//aaAaaAA
        addRequirements(intakeVision, drive);
//AAaaAAaAAAaAaaaaAa

//AAaaaaaaAaAAaAaAAAa
    }
//aaAAAaAaAAaaaaaA

//AaaAaAaaaaaaaAaaAA
    @Override
//AAaaaaAaaaAA
    public void initialize(){
//AaaaaAAAAaA
        yProcessor.reset(yPercent.getAsDouble());
//AaAaaa
    }
//aAaaAAAAAaAA

//AaAAAAaaaAa
    @Override
//AaAAaAAaAAAaaaaaAA
    public void execute() {
//aAaAAaAaaaaAaAaA

//AaAAaaAaAAAA
        if (DriveConstants.intakeVisionKP.hasChanged())
//AAAAAaaAAaaaAaA
            controller.setP(DriveConstants.intakeVisionKP.get());
//AAAaAAAAa
        if (DriveConstants.intakeVisionKD.hasChanged())
//AAAaaA
            controller.setD(DriveConstants.intakeVisionKD.get());
//aaaaAaAAa

//AAAAaaaAAAaaAAaa
        if (intakeVision.hasTarget()) {
//aaAaAAAAaaAAAAaaaA
            double omegaOutput = controller.calculate(-intakeVision.getTX(), 0);
//AAaAAAaAaAAaAAaaaaa
            double yOutput = yProcessor.processJoystickInputs(yPercent.getAsDouble()) * DriveConstants.maxSpeedMPerS;
//AaAaaaAAaAAAaAaaA
            ChassisSpeeds targetSpeeds = new ChassisSpeeds(0, yOutput, omegaOutput);
//aaaaAAaaaaAA

//AAaaAaaAaaaAaaaaA
            drive.setGoalChassisSpeeds(targetSpeeds);
//AAAaAaaaa

//AaaaAa
        }
//aaAAAaAAAaAa

//AAaAaAAAAa
    }
//AaaaaaAaaAaA
    
//AaaaaaA
}
