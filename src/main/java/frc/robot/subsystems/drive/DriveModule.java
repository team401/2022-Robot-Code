//AaaaaAAA
package frc.robot.subsystems.drive;
//AaaAaA

//AAAAAaAAAaaaaAA
import com.ctre.phoenix.motorcontrol.ControlMode;
//aaAAAaAaAaaaaaaAA
import com.ctre.phoenix.motorcontrol.DemandType;
//AAaAAaAAAaAaAaAAA
import com.ctre.phoenix.motorcontrol.NeutralMode;
//aaAAaaaAaAaaAaa
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
//AaAAAaaaAaaaaaAaAAA
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//AaaAAaAaaaAaA
import com.ctre.phoenix.sensors.CANCoder;
//aAAaa
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
//aAAAaaAAaAAAAa

//AaAAaAAaaaA
import edu.wpi.first.math.util.Units;
//AAAaaaaaAaAaaAa
import frc.robot.Constants;
//aAAAaAAaaAAAAAaAAaa
import frc.robot.Constants.DriveConstants;
//aAAAaA

//aAaAAaAaa
public class DriveModule {
//aaAaAAaAAAAAAaAaAa

//AAaaAAa
    public double driveVelocityRadPerS;
//AaAAAaaAAAAAaAaAaAa
    public double drivePositionRad;
//aaAaAaaAAaa
    public double rotationPositionRad;
//aAAAAAAaAa

//aAAaAAAAAAaa
    private final TalonFX driveMotor;
//aAaAaA
    private final TalonFX rotationMotor;
//AaAAaaaa
    private final CANCoder rotationEncoder;
//aAAAAaaAAAaAaaaaaA
    private final double initialOffsetRadians;
//aAaAaAa

//AaaaAaaaAaAAAA
    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
//aAAAaAAaA
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
//aaaaAaaAaAaAA
        //if (!needMotorSensor) {
//AaaAaaa
        //    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
//aAaAaaAaAa
        //}
//AAAAAaAaAAAA
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
//aaaaA
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
//aAaAa
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
//aAAaaA
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
//aaaaAa
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
//aaaAaaaA
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
//aAaaaaa
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
//aaAAAAaaaAaaAaaaAA
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
//aAAaAaaaaaAAaaaA
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
//AaAAaAaaa
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
//aaaAAaaAAaAAAaAAa
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);
//AaAaaAAaAaA
    }
//aAaaaaAAaAAAaAaAa

//AAAAA
    public DriveModule(int driveMotorID, int rotationMotorID, int cancoderID, double measuredOffsetsRadians) {
//aaAAAaAAaaAaaA

//AAaaAaaA
        driveMotor = new TalonFX(driveMotorID, Constants.canivoreName);
//aaaaAAAaaa
        rotationMotor = new TalonFX(rotationMotorID, Constants.canivoreName);
//AAAaAAaaaaAa
        rotationEncoder = new CANCoder(cancoderID, Constants.canivoreName);
//aaaaaaaaAAA
        
//aAaAaAAaAAaaAAaaa

//AAAAaAa
        driveMotor.configFactoryDefault(1000);
//aaAAa
        rotationMotor.configFactoryDefault(1000);
//AAAaaAAAAAAA
        setFramePeriods(driveMotor, true);
//aAAAaaaAa
        setFramePeriods(rotationMotor, false);
//aAAAaA

//AaAAaaAAA
        driveMotor.setNeutralMode(NeutralMode.Brake);
//aAAAAAAAAAaaaaaaA
        rotationMotor.setNeutralMode(NeutralMode.Brake);
//AAAAaaAaAAA

//AaaAaaaAaaaAAaaA
        driveMotor.setInverted(true);
//AaAAAAaaAaaA
        
//aAaAaAAAaAa
        driveMotor.configVoltageCompSaturation(12, 1000);
//aaAaAAaaaaAAaaAaaa
        driveMotor.enableVoltageCompensation(true);
//AaAAAaAaaaaAAaAAa
        rotationMotor.configVoltageCompSaturation(12, 1000);
//aaaaA
        rotationMotor.enableVoltageCompensation(true);
//AAaaaaaAa

//aAaAaaAAAaaAaAA
        driveMotor.configNeutralDeadband(0, 1000);
//AAAAAAAaAAaAaA
        rotationMotor.configNeutralDeadband(0, 1000);
//AaAAA

//aAAaaA
        rotationEncoder.configFactoryDefault(1000);
//AaAAaAAaAaAAa
        rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20, 1000);
//AaaaaaAAAAa
        rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255, 1000);
//aaAAa

//AaAaAa
        initialOffsetRadians = measuredOffsetsRadians;
//aaAaaAAAAa

//aaaaAaa
    }
//AaaAAAA

//AAaAAAA
    public void updateVariables() {
//AaaAaAaaaa
        
//AAAaaAAAaaAaa
        drivePositionRad = driveMotor.getSelectedSensorPosition() 
//aAaAA
            / 2048.0 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction;
//aaaAAaaaAaAa

//AAaaAaAAAAaAaAaAaAA
        driveVelocityRadPerS = driveMotor.getSelectedSensorVelocity() 
//AaAAAaaaA
            / 2048.0 * 10.0 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction;
//aAaAAAAA
            
//AaAAAAaAa
        //Using relative encoder in the CANCoder
//aAaaaaaAAaAAAAA
        rotationPositionRad = Units.degreesToRadians(rotationEncoder.getPosition()) - initialOffsetRadians;
//AAaAaAAaAaaAaaaaaaa

//AAaaaAaaAAAA
    }
//AAaAaAaAA

//AaaaAaaAaaaAAAaAA
    public void zeroEncoders() {
//AaaAaAaAaAAAaaAAAA
        rotationEncoder.setPositionToAbsolute(1000);
//AAaAAAaAaaaaAaAA
        driveMotor.setSelectedSensorPosition(0, 0, 1000);
//aAAaAaaAAaaaAA
    }
//aAAAAaaaaaAAa

//aAaAAaAAAaaaa
    public void setRotationVoltage(double volts) {
//aAaaAa
        rotationMotor.set(ControlMode.PercentOutput, volts/12);
//aAaAAAaAaAaaAaaAAA
    }
//AaaaAa

//AAAaaaaaaAAaAa
    public void setDriveVoltage(double volts) {
//AAAaAAAaaa
        driveMotor.set(ControlMode.PercentOutput, volts/12);
//AAaAaaa
    }
//AaAaaaaaAaAAaAaA

//AaaAaaaA
    public void setDriveVelocity(double velocityRadPerS, double ffVolts) {
//AAaaAaAaaAaAaaaAaAa
        // Convert rad/s to motor velocity
//AaaaAaAaa
        double velocityTicksPer100ms = velocityRadPerS * 2048.0 / 10.0 / 2.0 / Math.PI * DriveConstants.driveWheelGearReduction;
//aaaAaAAaaAAAa

//aaAAAaAaaaaAAAAA
        driveMotor.set(ControlMode.Velocity, velocityTicksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12.0);
//AAAAA
    }
//aaAaaaaAAAA

//aaAaAaAAAaaaaa
    public void setDrivePD(double p, double d) {
//aAAaaaaAaAAaaAaaAAA
        driveMotor.config_kP(0, p);
//aAaAAAAaaaAaAAaaAAa
        driveMotor.config_kD(0, d);
//AaAaaaAaaaAAaaaaaaa
    }
//AaaaAAaaaAa
    
//AaAaAAAaaaaaAAAAa
}
