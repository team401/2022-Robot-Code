//AAaAaAAAaaaaAAaaAA
package frc.robot.subsystems;
//aAaAaAaaaAAAa

//AaaAAaAAaaaaaAaA
import com.revrobotics.CANSparkMax;
//aAAAaAAAAAaA
import com.revrobotics.CANSparkMax.IdleMode;
//aAaaAAAaaaAAa
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//aAaAaaAaAAAaaA

//AaAAaA
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//aAaAaAA
import frc.robot.Constants.CANDevices;
//aaAaaAa

//AaAaaaaa
public class IntakeWheels extends SubsystemBase {
//AaAAaaaAAAaaAaAaAAA

//aAaaaaaAaaaaaa
    private final CANSparkMax intakeMotor;
//AaAAAA

//AAaaAAaAaAAAAAaaa
    public IntakeWheels() {
//aaaaaAaAaaaaAAAaaaA
        intakeMotor = new CANSparkMax(CANDevices.intakeMotorID, MotorType.kBrushed);
//AAaAaAaAaaa
        intakeMotor.setIdleMode(IdleMode.kCoast);
//AaAaAaAA
        intakeMotor.setInverted(false);
//aAAaaAaAaA
    }
//aAaAaaaaa

//aAAaaAaAaaAa
    public void setPercent(double percent) {
//AaaaAA
        intakeMotor.set(percent);        
//aAAAaAaaAAaA
    }
//AaaAaAAAaaAAAa
    
//AaaaAaAaaaaaaaAaaAa
}
