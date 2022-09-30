package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

public class IntakeWheels extends SubsystemBase {

    private final CANSparkMax intakeMotor;

    public IntakeWheels() {
        intakeMotor = new CANSparkMax(CANDevices.intakeMotorID, MotorType.kBrushed);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setInverted(false);
    }

    public void setPercent(double percent) {
        intakeMotor.set(percent);        
    }
    
}
