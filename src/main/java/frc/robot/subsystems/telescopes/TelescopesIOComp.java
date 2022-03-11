package frc.robot.subsystems.telescopes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

public class TelescopesIOComp implements TelescopesIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public TelescopesIOComp() {
        leftMotor = new CANSparkMax(CANDevices.leftTelescopingMotorID, MotorType.kBrushed);
        rightMotor = new CANSparkMax(CANDevices.leftTelescopingMotorID, MotorType.kBrushed);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftEncoder = leftMotor.getEncoder(Type.kQuadrature, 4096);
        rightEncoder = rightMotor.getEncoder(Type.kQuadrature, 4096);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

    }

    @Override
    public void updateInputs(TelescopesIOInputs inputs) {
        inputs.leftPositionIn = leftEncoder.getPosition() * ClimberConstants.linearConversion;
        inputs.rightPositionIn = rightEncoder.getPosition() * ClimberConstants.linearConversion;
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
    }

    @Override
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);        
    }

    @Override
    public void setLeftVolts(double volts) {
        leftMotor.setVoltage(volts);        
    }

    @Override
    public void setRightVolts(double volts) {
        rightMotor.setVoltage(volts);        
    }
    
}
