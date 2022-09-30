package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private final TalonFX turretMotor = new TalonFX(CANDevices.turretMotorID, Constants.canivoreName);

    private final DutyCycleEncoder turretAbsEncoder = new DutyCycleEncoder(DIOChannels.turretEncoderPulse);
    private final Encoder turretRelEncoder = new Encoder(DIOChannels.turretEncoderA, DIOChannels.turretEncoderB, false);

    public double absolutePositionRad;
    public double positionRad;
    public double velocityRadPerS;
    public double current;

    private PIDController positionController = new PIDController(TurretConstants.positionKp.get(), 0, TurretConstants.positionKd.get());
    private Rotation2d goalPosition = new Rotation2d();
    private double velocityGoal = 0;

    private double encoderOffset = 0;
    private int setupCycleCount = 0;

    private boolean killed = false;
    private double lastUpdateValue = 0;
    private long lastUpdateTimeMS = System.currentTimeMillis();
    
    private boolean zeroOverride = false;

    private final Timer overdrawTimer = new Timer();

    public Turret() {

        turretMotor.configFactoryDefault(1000);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(true);
        turretMotor.configVoltageCompSaturation(12, 1000);
        turretMotor.enableVoltageCompensation(true);
        turretMotor.configNeutralDeadband(0, 1000);
 
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);
 
        turretAbsEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        turretRelEncoder.setDistancePerPulse(1 / 2048.0);

        resetEncoder();
        
        positionController.setTolerance(Units.degreesToRadians(3));
        
        lastUpdateTimeMS = System.currentTimeMillis();

    }

    @Override
    public void periodic() {

        absolutePositionRad = Units.rotationsToRadians(-turretAbsEncoder.getDistance()) - TurretConstants.turretEncoderOffsetRad;
        positionRad = Units.rotationsToRadians(turretRelEncoder.getDistance());
        velocityRadPerS = Units.rotationsToRadians(turretRelEncoder.getRate());
        current = turretMotor.getSupplyCurrent();

        if (setupCycleCount == TurretConstants.setupCycleCount) {
            resetEncoder();
            encoderOffset = MathUtil.angleModulus(absolutePositionRad);
            setupCycleCount++;
            lastUpdateValue = positionRad + encoderOffset;
        }
        else {
            setupCycleCount++;
        }
        
        if (!DriverStation.isEnabled()) {
            setNeutralMode(NeutralMode.Coast);
        }
        else {
            setNeutralMode(NeutralMode.Brake);
        }
        
        // Update gains if they have changed
        if (TurretConstants.positionKp.hasChanged() || TurretConstants.positionKd.hasChanged()) {
            positionController.setP(TurretConstants.positionKp.get());
            positionController.setD(TurretConstants.positionKd.get());
        }
        
        if (TurretConstants.velocityKp.hasChanged() || TurretConstants.velocityKd.hasChanged()) {
            setVelocityPD(TurretConstants.velocityKp.get(), TurretConstants.velocityKd.get());
        }
        
        double turretRotation = positionRad + encoderOffset;

        if (absolutePositionRad != lastUpdateValue) {
            lastUpdateTimeMS = System.currentTimeMillis();
        }
        lastUpdateValue = absolutePositionRad;

        if (System.currentTimeMillis()-lastUpdateTimeMS > 500 && setupCycleCount > TurretConstants.setupCycleCount && DriverStation.isEnabled())
            killed = true;
        
        if (Math.abs(turretRotation) > TurretConstants.turretLimitUpper + Math.PI/2 && setupCycleCount > TurretConstants.setupCycleCount && DriverStation.isEnabled())
            killed = true;
        
        //PID control - equivalent of our old setdesiredpositionclosedloop methods continuously
        double output = positionController.calculate(turretRotation, zeroOverride ? 0 : goalPosition.getRadians());
        // Only add feed velocity if we are not at our hard stops
        if (goalPosition.getRadians() > TurretConstants.turretLimitLower && goalPosition.getRadians() < TurretConstants.turretLimitUpper) {
            output += TurretConstants.turretModel.calculate(velocityGoal);
        }
        if (setupCycleCount > TurretConstants.setupCycleCount && !killed)
            setVoltage(output);
        else
            setVoltage(0);

        RobotState.getInstance().recordTurretObservations(new Rotation2d(turretRotation), velocityRadPerS);
    }

    public void resetEncoder() {
        turretRelEncoder.reset();
    }

    public void setVoltage(double voltage) {
        turretMotor.set(ControlMode.PercentOutput, voltage/12);
    }
    
    public void setVelocitySetpoint(double velocityRadPerS, double ffVolts) {
        double velocityTicksPer100ms = velocityRadPerS / 10.0 / 2.0 / Math.PI * TurretConstants.turretGearRatio;
        turretMotor.set(ControlMode.Velocity, velocityTicksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12);
    }

    public void setVelocityPD(double p, double d) {
        turretMotor.config_kP(0, p, 1000);
        turretMotor.config_kD(0, d, 1000);
    }

    public double getCurrent() {
        return turretMotor.getStatorCurrent();
    }

    public void setNeutralMode(NeutralMode mode) {
        turretMotor.setNeutralMode(mode);
    }

    public void setPositionGoal(Rotation2d goal, double velocity) {

        velocityGoal = velocity;
        double goalWrapped = MathUtil.angleModulus(goal.getRadians());
        
        //clamps max values to be within -90 and 90 deg
        goalWrapped = MathUtil.clamp(goalWrapped, TurretConstants.turretLimitLower, TurretConstants.turretLimitUpper);
        this.goalPosition = new Rotation2d(goalWrapped);
    }

    public void setPositionGoal(Rotation2d goal) {

        setPositionGoal(goal, 0);

    }

    public double getVelocityRadPerS() {
        return velocityRadPerS;
    }

    public boolean atGoal() {

        return positionController.atSetpoint();

    }

    public void setZeroOverride(boolean zero) {
        zeroOverride = zero;
    }

    public void kill() {
        killed = true;
    }

    public void unkill() {
        killed = false;
    }
    
}
