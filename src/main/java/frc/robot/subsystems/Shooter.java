package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    public double flywheelSpeedRadPerS;
    public double hoodPositionRad;
    public double hoodCurrent;
    public double hoodVelocity;

    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;

    private final CANSparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkMaxPIDController hoodController;

    private double flywheelGoalRadPerS = 0;
    private double hoodGoalRad = 0;
    private boolean homed = false;
    private boolean hoodEnable = false;
    private boolean flywheelEnable = false;
    private final Timer homeTimer = new Timer();
    
    private final double flywheelToleranceRadPerS = Units.rotationsPerMinuteToRadiansPerSecond(150);
    private final double hoodToleranceRad = Units.rotationsToRadians(0.25);
    
    private double rpmOffset = 60;
    
    private static boolean atGoal = false;
    
    private static double shooterGoal = 0;

    public Shooter() {
        leftShooterMotor = new TalonFX(CANDevices.leftShooterMotorID);
        rightShooterMotor = new TalonFX(CANDevices.rightShooterMotorID);

        hoodMotor = new CANSparkMax(CANDevices.hoodMotorID, MotorType.kBrushless);

        rightShooterMotor.configFactoryDefault();
        leftShooterMotor.configFactoryDefault();

        rightShooterMotor.configNeutralDeadband(0);
        leftShooterMotor.configNeutralDeadband(0);

        hoodMotor.restoreFactoryDefaults();

        rightShooterMotor.setInverted(TalonFXInvertType.OpposeMaster);
        hoodMotor.setInverted(true);

        rightShooterMotor.follow(leftShooterMotor);
        rightShooterMotor.setStatusFramePeriod(1, 255);
        rightShooterMotor.setStatusFramePeriod(2, 255);

        leftShooterMotor.configVoltageCompSaturation(12, 1000);
        leftShooterMotor.enableVoltageCompensation(true);

        // Current Limits
        hoodMotor.setSmartCurrentLimit(30);

        // sets up hood PID Controller
        hoodController = hoodMotor.getPIDController();

        hoodEncoder = hoodMotor.getEncoder();

        SmartDashboard.putNumber("Hood Desired", 0.27);
        SmartDashboard.putNumber("Shooter Desired", 0);
    }

    @Override
    public void periodic() {
        long m_Start = System.currentTimeMillis();

        flywheelSpeedRadPerS = leftShooterMotor.getSelectedSensorVelocity() * 2.0 * Math.PI * 10.0 / 2048.0;
        hoodPositionRad = hoodEncoder.getPosition() * 2.0 * Math.PI / ShooterConstants.hoodRackRatio
                + ShooterConstants.hoodOffsetRad;
        hoodCurrent = hoodMotor.getOutputCurrent();
        hoodVelocity = hoodEncoder.getVelocity();

        if (ShooterConstants.hoodKp.hasChanged() || ShooterConstants.hoodKd.hasChanged()) {
            setHoodPD(ShooterConstants.hoodKp.get(), ShooterConstants.hoodKd.get());
        }

        if (ShooterConstants.flywheelKp.hasChanged() || ShooterConstants.flywheelKd.hasChanged()) {
            setFlywheelPD(ShooterConstants.flywheelKp.get(), ShooterConstants.flywheelKd.get());
        }

        SmartDashboard.putNumber("RPM Offset", rpmOffset);

        if (!homed) {
            if (DriverStation.isEnabled()) {
                if (Math.abs(hoodVelocity) < ShooterConstants.hoodHomingThresholdRadPerS) {
                    homeTimer.start();
                } else {
                    homeTimer.stop();
                    homeTimer.reset();
                }

                if (homeTimer.hasElapsed(ShooterConstants.hoodHomingTimeS)) {
                    homed = true;
                    hoodEnable = false;
                    zeroHoodEncoder();
                    setHoodVoltage(0);
                } else {
                    setHoodVoltage(ShooterConstants.hoodHomingVolts);
                }
            }
        } else {
            if (hoodEnable) {
                setHoodPositionSetpoint(hoodGoalRad);
            } else {
                setHoodVoltage(0);
            }

            if (flywheelEnable) {
                setFlywheelVelocity(flywheelGoalRadPerS, ShooterConstants.flywheelModel.calculate(flywheelGoalRadPerS));
            } else {
               setFlywheelVoltage(0);
            }
        }

        atGoal = atGoal();

        SmartDashboard.putNumber("Flywheel Speed", Units.radiansPerSecondToRotationsPerMinute(flywheelSpeedRadPerS));
        SmartDashboard.putNumber("Flywheel Goal", Units.radiansPerSecondToRotationsPerMinute(flywheelGoalRadPerS));
        SmartDashboard.putNumber("Distance", RobotState.getInstance().getAimingParameters().getDistanceM());
    }

    public void zeroHoodEncoder() {
        hoodEncoder.setPosition(0);
    }

    public void setHoodPositionSetpoint(double angleRad) {
        double motorRevs = (angleRad - ShooterConstants.hoodOffsetRad) / 2.0 / Math.PI * ShooterConstants.hoodRackRatio;
        hoodController.setReference(motorRevs, ControlType.kPosition);
    }

    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    public void setHoodPD(double p, double d) {
        hoodController.setP(p);
        hoodController.setD(d);
    }

    public void setFlywheelVelocity(double velocityRadPerS, double ffVolts) {
        double flywheelTicksPer100ms = velocityRadPerS / 2.0 / Math.PI / 10.0 * 2048.0;
        leftShooterMotor.set(ControlMode.Velocity, flywheelTicksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12.0);
    }

    public void setFlywheelVoltage(double volts) {
        leftShooterMotor.set(ControlMode.PercentOutput, volts / 12.0);
    }

    public void setFlywheelPD(double p, double d) {
        leftShooterMotor.config_kP(0, p, 1000);
        leftShooterMotor.config_kD(0, d, 1000);
    }
    
    public double getHoodVelocity() {
        return hoodVelocity;
    }

    public void setSetpoint(double hoodAngleRad, double flywheelGoalRadPerS) {
        hoodEnable = true;
        flywheelEnable = true;
        this.hoodGoalRad = hoodAngleRad;
        this.hoodGoalRad = MathUtil.clamp(this.hoodGoalRad, ShooterConstants.hoodMinRad, ShooterConstants.hoodMaxRad);
        this.flywheelGoalRadPerS = flywheelGoalRadPerS;
        if (flywheelGoalRadPerS != 0) this.flywheelGoalRadPerS += Units.rotationsPerMinuteToRadiansPerSecond(rpmOffset);
        shooterGoal = flywheelGoalRadPerS;
    }

    public void stopShooter() {
        hoodEnable = false;
        flywheelEnable = false;
    }

    public void setFlywheelVolts(double volts) {
        setFlywheelVoltage(volts);
    }

    public double getFlywheelVelocityRadPerS() {
        return flywheelSpeedRadPerS;
    }

    public double getHoodPositionRad() {
        return hoodPositionRad;
    }

    public boolean atGoal() {
        return Math.abs(flywheelSpeedRadPerS-flywheelGoalRadPerS) < flywheelToleranceRadPerS &&
                Math.abs(hoodPositionRad-hoodGoalRad) < hoodToleranceRad;
    }

    public void killTurret() {
        setFlywheelVolts(0);
        flywheelEnable = false;
    }

    public void killHood() {
        setHoodVoltage(0);
        hoodEnable = false;
    }

    public void incrementRPMOffset(int offset) {
        rpmOffset += offset;
    }

    public static boolean atGoalStatic() {
        return atGoal && shooterGoal != 0;
    }
    
}
