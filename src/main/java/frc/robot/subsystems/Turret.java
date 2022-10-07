//AaaaaaAAaaaaaAAaa
package frc.robot.subsystems;
//AaaAAaAaaAa

//aAAaAaAaAAaAaAA
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//AAaAAAAAAaa

//aaAaaaAA
import com.ctre.phoenix.motorcontrol.ControlMode;
//aaaaaAAAa
import com.ctre.phoenix.motorcontrol.DemandType;
//AAAaaaAAAA
import com.ctre.phoenix.motorcontrol.NeutralMode;
//aAaaaAAAaAA
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
//AaAAAAaAAaa
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
//aAaaaaaA
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
//AaaAaaA
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//AAAaAAAAAAAaAa
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//aaAaAa
import com.ctre.phoenix.sensors.CANCoder;
//aAAaaaaaAaAAA
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
//aAaaAaaAAAaAA

//AAAaAaAaaaaAaaaaA
import edu.wpi.first.math.util.Units;
//aaaAaaAaaaaaA
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//aaAAAAaAAaAa
import edu.wpi.first.wpilibj.Encoder;
//aAaAaAAA
import frc.robot.Constants.TurretConstants;
//aaaaaAaaaAaAaaAAaA
import frc.robot.Constants;
//aaaAAaAaAA
import frc.robot.Constants.CANDevices;
//aaAaAaaAa
import frc.robot.Constants.DIOChannels;
//AaaaAAaaa

//AaAaAAaAAAaAaaAaAaA
import com.ctre.phoenix.motorcontrol.NeutralMode;
//AaaaAaaa

//aaaAaAA
import com.ctre.phoenix.motorcontrol.NeutralMode;
//aaAAaaaAA
import edu.wpi.first.math.MathUtil;
//AaaaaAaAaaa
import edu.wpi.first.math.geometry.Rotation2d;
//AAaaaAaaAAAAaAAaA
import edu.wpi.first.math.util.Units;
//AaAAAaaAa
import edu.wpi.first.wpilibj.DriverStation;
//AAAaaaAA
import edu.wpi.first.wpilibj.Timer;
//aAAaAAAAaaAa
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//aaaAaAAAAaaAaAaA

//AAaaaaaAAaAAAaaa
import edu.wpi.first.math.controller.PIDController;
//aaAAaaaAAaaaAAAaaaa
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//AaAAAAaaAA
import frc.robot.RobotState;
//aaAaAAAAAAAAaA
import frc.robot.Constants.TurretConstants;
//AaaaAAaaAAAaAa

//aAaAAAaAaaAaAAaaa
public class Turret extends SubsystemBase {
//aaAAaAAaaA

//aaaaaA
    private final TalonFX turretMotor = new TalonFX(CANDevices.turretMotorID, Constants.canivoreName);
//aaAaAAaa

//aaAaaaAaAaaa
    private final DutyCycleEncoder turretAbsEncoder = new DutyCycleEncoder(DIOChannels.turretEncoderPulse);
//aaAAaAaAaAAaaaaaaa
    private final Encoder turretRelEncoder = new Encoder(DIOChannels.turretEncoderA, DIOChannels.turretEncoderB, false);
//aAaaaaAaaAa

//aaAAAAaAAaAA
    public double absolutePositionRad;
//AAaAAA
    public double positionRad;
//AAAaaaaaaAAAAaAa
    public double velocityRadPerS;
//AaaaAaaaAaAaAAAaaA
    public double current;
//aAaaAa

//AaaAAaaAaaAAA
    private PIDController positionController = new PIDController(TurretConstants.positionKp.get(), 0, TurretConstants.positionKd.get());
//aaAAaaAAAa
    private Rotation2d goalPosition = new Rotation2d();
//AAaAAaAaAAAAAAAAa
    private double velocityGoal = 0;
//AAaaaaAaaaAaaAaaaaa

//aaaAAaAAAaAAaa
    private double encoderOffset = 0;
//AAAAaaAAAAa
    private int setupCycleCount = 0;
//AaaaAAaAaaaAaA

//AaaAaaaaa
    private boolean killed = false;
//aaAaA
    private double lastUpdateValue = 0;
//AaAaaaaA
    private long lastUpdateTimeMS = System.currentTimeMillis();
//AAAAAaAaAAaaAaaaa
    
//aaaaAAAAaaa
    private boolean zeroOverride = false;
//AaAaaaAAAa

//AAAAaAaa
    private final Timer overdrawTimer = new Timer();
//AaaaaaAaaa

//AAAaAAaaAaAa
    public Turret() {
//aaaAaA

//aaaaAaAAaaAaAAaAaaA
        turretMotor.configFactoryDefault(1000);
//aAAAaa
        turretMotor.setNeutralMode(NeutralMode.Brake);
//AAAAAaAAaaAA
        turretMotor.setInverted(true);
//aaaAA
        turretMotor.configVoltageCompSaturation(12, 1000);
//aAaAaaAAaAa
        turretMotor.enableVoltageCompensation(true);
//AAaaAaAaA
        turretMotor.configNeutralDeadband(0, 1000);
//AAAAA
 
//AaAaaaaAAaaaAAAaAa
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
//aaAAaaAA
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
//aaaaAAAaAAAAaAaaaA
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
//aAAAAAAAaAAaaaa
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
//aaAAAAaAa
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
//AAAAaaaaaaAAAa
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
//AaAaaaa
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
//aAaAAaAaaAaaaA
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
//AaAAAaAaAaAaAA
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
//AaAAaaAaaAaaAaAaaa
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
//aaaaaaaaAAAaaA
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
//AAAaaAaaAa
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);
//AAaaaaAA
 
//AAAAa
        turretAbsEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
//AAaaAAAAaAaaa
        turretRelEncoder.setDistancePerPulse(1 / 2048.0);
//AaAaaAAaaaa

//aaAAAa
        resetEncoder();
//aaaAAaAaaaaAaAA
        
//AAaaAaaAAaaAa
        positionController.setTolerance(Units.degreesToRadians(3));
//aaaaaaaAaAAA
        
//aAAaaAaA
        lastUpdateTimeMS = System.currentTimeMillis();
//aAAaaAaAAAAaaaaAaa

//AAAaAAaAAa
    }
//AaaAaAAAAaaAaa

//aAaAaaaaaAaaAAA
    @Override
//aAaAaA
    public void periodic() {
//AaaAaAAAAaAAaAa

//AAaAaaAA
        absolutePositionRad = Units.rotationsToRadians(-turretAbsEncoder.getDistance()) - TurretConstants.turretEncoderOffsetRad;
//aaaAAaAaAaAaAAAaA
        positionRad = Units.rotationsToRadians(turretRelEncoder.getDistance());
//AAaaaaaaaA
        velocityRadPerS = Units.rotationsToRadians(turretRelEncoder.getRate());
//aAaAaaAA
        current = turretMotor.getSupplyCurrent();
//AaAAaaAAAaaAAAAA

//aAaAaa
        if (setupCycleCount == TurretConstants.setupCycleCount) {
//AaAAaAaAAaaAaaA
            resetEncoder();
//aAaaAAaAAaaAAAaaaAA
            encoderOffset = MathUtil.angleModulus(absolutePositionRad);
//aAaAAAaa
            setupCycleCount++;
//aAAAAAaaAAaAA
            lastUpdateValue = positionRad + encoderOffset;
//aaAaaaAAAaaaaa
        }
//AAAaA
        else {
//AaaAAAAaaaAaAAaA
            setupCycleCount++;
//aaAAAaaaaAAaA
        }
//AaAAAaAA
        
//aaAAAAAA
        if (!DriverStation.isEnabled()) {
//AAAAAA
            setNeutralMode(NeutralMode.Coast);
//AAAaAA
        }
//AAAaaaaaaaaAaAAAAAA
        else {
//AaAaaaaaAaaaaaAAa
            setNeutralMode(NeutralMode.Brake);
//AaAAAaaAAAAA
        }
//AaAAaAaAaaaAAaaaaAA
        
//AaaaAAaaaa
        // Update gains if they have changed
//aaAaaAaaAAaaAaaa
        if (TurretConstants.positionKp.hasChanged() || TurretConstants.positionKd.hasChanged()) {
//AaAaAaAAaAA
            positionController.setP(TurretConstants.positionKp.get());
//AAaAAAaaAAaaAaaaaA
            positionController.setD(TurretConstants.positionKd.get());
//AAaAaAAaAAAaA
        }
//aaAAaAaAAAaAaA
        
//aAaaaaa
        if (TurretConstants.velocityKp.hasChanged() || TurretConstants.velocityKd.hasChanged()) {
//aaAAAAAAAAAA
            setVelocityPD(TurretConstants.velocityKp.get(), TurretConstants.velocityKd.get());
//aAAAAaAaaaAA
        }
//aaaAAaAaaaAaAAAAAa
        
//aAaAaA
        double turretRotation = positionRad + encoderOffset;
//aAaaAAaaaaaAaAAAa

//aaAAaa
        if (absolutePositionRad != lastUpdateValue) {
//aAaAAaA
            lastUpdateTimeMS = System.currentTimeMillis();
//AAaaAAaAaaaA
        }
//aAAaAaa
        lastUpdateValue = absolutePositionRad;
//aAaaAaAaAaaaAAAAAAa

//aaAaAaaAaa
        if (System.currentTimeMillis()-lastUpdateTimeMS > 500 && setupCycleCount > TurretConstants.setupCycleCount && DriverStation.isEnabled())
//aaaaAAAAAaA
            killed = true;
//aAAaaaa
        
//AAaAA
        if (Math.abs(turretRotation) > TurretConstants.turretLimitUpper + Math.PI/2 && setupCycleCount > TurretConstants.setupCycleCount && DriverStation.isEnabled())
//aAaaAaaAAAAa
            killed = true;
//AaAAaaAAAaaAaaaAa
        
//aAaAaAaAaA
        //PID control - equivalent of our old setdesiredpositionclosedloop methods continuously
//aAAAAaAaaaAAAA
        double output = positionController.calculate(turretRotation, zeroOverride ? 0 : goalPosition.getRadians());
//aaAAAAaAAAaAAAaaa
        // Only add feed velocity if we are not at our hard stops
//AAAaAA
        if (goalPosition.getRadians() > TurretConstants.turretLimitLower && goalPosition.getRadians() < TurretConstants.turretLimitUpper) {
//AAaAaAaAAAaaaA
            output += TurretConstants.turretModel.calculate(velocityGoal);
//AAaaaaAAaaaAaAaA
        }
//AaaaaaAAAAaaaaaaaA
        if (setupCycleCount > TurretConstants.setupCycleCount && !killed)
//AaaAaaAAAaaAaAaa
            setVoltage(output);
//AAaAaaaaa
        else
//aAAAAAaaaAaaAaAAa
            setVoltage(0);
//AaAAAAAaaAaAAA

//AaaAAaA
        RobotState.getInstance().recordTurretObservations(new Rotation2d(turretRotation), velocityRadPerS);
//aAaAaaAAAAa
    }
//AAAAAAAAaAAaaAaaaa

//AAaaaA
    public void resetEncoder() {
//AAaaAAaaAaaaa
        turretRelEncoder.reset();
//AAAAaAAaaaAAaAAA
    }
//aAaaAAAAAAAAAaaaAaA

//AAAaAAA
    public void setVoltage(double voltage) {
//aaAaaaAAaAA
        turretMotor.set(ControlMode.PercentOutput, voltage/12);
//aAaAAAAAa
    }
//aaaaAa
    
//aAaAAAaa
    public void setVelocitySetpoint(double velocityRadPerS, double ffVolts) {
//AAAAaAAaaaAAAAaAa
        double velocityTicksPer100ms = velocityRadPerS / 10.0 / 2.0 / Math.PI * TurretConstants.turretGearRatio;
//AaaaAaaAAaAAA
        turretMotor.set(ControlMode.Velocity, velocityTicksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12);
//AaAaAaAaaaAAaAaaaaa
    }
//aaAAaA

//AAAAAaAAA
    public void setVelocityPD(double p, double d) {
//AaAaaaaAAAAaAA
        turretMotor.config_kP(0, p, 1000);
//AaAAAaaAaaAAaaAA
        turretMotor.config_kD(0, d, 1000);
//AaaAAAaaAAAaAAAaAaa
    }
//AaAAaAaAaaAAaaAaa

//aAaAAaAa
    public double getCurrent() {
//AAAaaaaa
        return turretMotor.getStatorCurrent();
//aaaAaaa
    }
//AaaAaaaaAaaaa

//AAAAAaaaAaaA
    public void setNeutralMode(NeutralMode mode) {
//aaaaAAaaAaAA
        turretMotor.setNeutralMode(mode);
//aAaaaaAA
    }
//aaAAAaAaaaaaaAAaA

//aAAaaaaAAaAaAAaAaA
    public void setPositionGoal(Rotation2d goal, double velocity) {
//aaaaaAaAAAaaaaaAAA

//AaAaAAAAAA
        velocityGoal = velocity;
//aaaAaaAaaaaaaAAA
        double goalWrapped = MathUtil.angleModulus(goal.getRadians());
//aAAAaaAAAAAAA
        
//aaaAAaaaA
        //clamps max values to be within -90 and 90 deg
//aaaaaAaaAaaAAaA
        goalWrapped = MathUtil.clamp(goalWrapped, TurretConstants.turretLimitLower, TurretConstants.turretLimitUpper);
//AaAAaaaAaAaAA
        this.goalPosition = new Rotation2d(goalWrapped);
//aAaAaaaaaaAaaaa
    }
//AAAaaaAA

//aAaAAAAAAAAaAaAAAAA
    public void setPositionGoal(Rotation2d goal) {
//AaAAaA

//aaaaaAaAAAaAaAaaa
        setPositionGoal(goal, 0);
//aAaAAaAA

//aaAAAaaaAaAaAAa
    }
//AAaAAaAaAAaaAaaaaAa

//AaAaAaAaaAaa
    public double getVelocityRadPerS() {
//aAAAAAAAaAaaaa
        return velocityRadPerS;
//AAaaaAaAAAAAaAaAAAA
    }
//AaAaaAaAaAaAaa

//AaaaaAA
    public boolean atGoal() {
//aaAAaAaAaAAAAAaaAAA

//aaAaaaaaaAAaA
        return positionController.atSetpoint();
//AaaaAaaaa

//aAAaaAAaAAaa
    }
//AAAaaAaAaaaAaa

//AaAAaaAaAaaAAaaAA
    public void setZeroOverride(boolean zero) {
//AaAaAaaAAAAa
        zeroOverride = zero;
//AaAaaAaAaAaAAAaa
    }
//aAaAAAaaAaaAaaAAAa

//AaaAaA
    public void kill() {
//aaAAaaaAAAaaa
        killed = true;
//AAAaAaaAaaa
    }
//AaaaaaaaaAAaA

//AAAAAaaAAa
    public void unkill() {
//AaAAaAaaAaAaaaAA
        killed = false;
//aAaAaAAAAAAaa
    }
//AaaAaaaaaAAaaA
    
//AaaAAaaaAaaAAAAAa
}
