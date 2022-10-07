//aAAaaAaaAaaaAaaaA
package frc.robot.subsystems;
//AaAaaAaaaaAAAAAAaA

//AaaAaAaAaAaaAaAaaa
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//AAaAaaAAAaaAAA

//aaAaaaaa
import com.ctre.phoenix.motorcontrol.ControlMode;
//AaaaaaA
import com.ctre.phoenix.motorcontrol.DemandType;
//AAAaaAaaAAAAAAAAAAA
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
//aaAaAaaAaAAAAAaaaaA
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//AAAAaAaAA
import com.revrobotics.CANSparkMax;
//aAaaaaaA
import com.revrobotics.RelativeEncoder;
//AAaAaaaaAAaaaaAaA
import com.revrobotics.SparkMaxPIDController;
//aaaAaAAaaAaAAAaAaAa
import com.revrobotics.CANSparkMax.ControlType;
//aAAaAAaaaa
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//AaaaaaAaAA

//AaaaAaaaaaAAAa
import frc.robot.Constants;
//AaaaAAaaAaaa
import frc.robot.Constants.CANDevices;
//AaAAaAaaAaaAAaaaaAA
import frc.robot.Constants.ShooterConstants;
//aAaaaaAaAaaA

//AaAaaAaAaaAaAAAA
import edu.wpi.first.math.MathUtil;
//aAaaAaA
import edu.wpi.first.math.util.Units;
//AaAAAaaaaAAaa
import edu.wpi.first.wpilibj.DriverStation;
//AAaaa
import edu.wpi.first.wpilibj.Timer;
//aAAaAAaaAAaaAAaa
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//AaaaaaA
import frc.robot.RobotState;
//aAaaaAaaaa
import frc.robot.Constants.ShooterConstants;
//AaAAAAaaa

//aaAaA
public class Shooter extends SubsystemBase {
//AAaaAaaa

//aaAaaAAAaAAaa
    public double flywheelSpeedRadPerS;
//AaaaAAAaAaAaAAA
    public double hoodPositionRad;
//aAaaAaA
    public double hoodCurrent;
//Aaaaa
    public double hoodVelocity;
//aAAAAaaAaAaaaAaAaaa

//aaAaAaaAaaaaA
    private final TalonFX leftShooterMotor;
//AaaaAAaaAaaa
    private final TalonFX rightShooterMotor;
//AaaAaAAaaaaa

//AaaaaAaAAAaAaaaAAa
    private final CANSparkMax hoodMotor;
//aaAAa
    private final RelativeEncoder hoodEncoder;
//AAAAaa
    private final SparkMaxPIDController hoodController;
//aaAaaAa

//aaaaAaaAAAAaaaAAAa
    private double flywheelGoalRadPerS = 0;
//aAaaAAAaAaaaAA
    private double hoodGoalRad = 0;
//AAAAAAaaaaAaaAAaaA
    private boolean homed = false;
//aAAAAaAaAAaAAAaaa
    private boolean hoodEnable = false;
//AaAAaaA
    private boolean flywheelEnable = false;
//AAAAAaAaaaAaAaAAa
    private final Timer homeTimer = new Timer();
//aAAaAAaaaA
    
//AAaaAaAAaaaaAa
    private final double flywheelToleranceRadPerS = Units.rotationsPerMinuteToRadiansPerSecond(150);
//aaaAaaA
    private final double hoodToleranceRad = Units.rotationsToRadians(0.25);
//aAaaaAa
    
//AAaAaaaAaa
    private double rpmOffset = 0;
//AAaAAAaAa
    
//AAaAaa
    private static boolean atGoal = false;
//aaaAaaaAaaaAA
    
//AaAAaAA
    private static double shooterGoal = 0;
//AaaAAAaaaA

//AaAAaaaa
    public Shooter() {
//aAAAAaaaAaAaAaAaAaA
        leftShooterMotor = new TalonFX(CANDevices.leftShooterMotorID);
//AaaaAaaaAAa
        rightShooterMotor = new TalonFX(CANDevices.rightShooterMotorID);
//AAAAaAaAaa

//AaAAAAAaAaAAAAaAa
        hoodMotor = new CANSparkMax(CANDevices.hoodMotorID, MotorType.kBrushless);
//aAaaAAaAAaaA

//AAAAaAaAaAAAAAAAaA
        rightShooterMotor.configFactoryDefault();
//aaAAAaaa
        leftShooterMotor.configFactoryDefault();
//AAAAAA

//aaaAAaaA
        rightShooterMotor.configNeutralDeadband(0);
//aAAAaaAaaAAa
        leftShooterMotor.configNeutralDeadband(0);
//aaaAAAaaaaAaaaa

//aAaAAAAaaaaaAa
        hoodMotor.restoreFactoryDefaults();
//aAaaa

//AaaaAAAaAa
        rightShooterMotor.setInverted(TalonFXInvertType.OpposeMaster);
//AaAaAAA
        hoodMotor.setInverted(true);
//aaAAaaa

//Aaaaaaaaaaaaa
        rightShooterMotor.follow(leftShooterMotor);
//AAaaAaAAaAAaAaa
        rightShooterMotor.setStatusFramePeriod(1, 255);
//AAaAaaAaAAAAa
        rightShooterMotor.setStatusFramePeriod(2, 255);
//AaAAAAAa

//AaaaaAAAaaAaAaAAA
        leftShooterMotor.configVoltageCompSaturation(12, 1000);
//aAAAa
        leftShooterMotor.enableVoltageCompensation(true);
//AaAaaaAAa

//aAAAa
        // Current Limits
//aAaAAaaAAaAA
        hoodMotor.setSmartCurrentLimit(30);
//AaAAAAaAaaaaaAaaaA

//aAaaaaAAaaaaaA
        // sets up hood PID Controller
//aaaaaAa
        hoodController = hoodMotor.getPIDController();
//AaaaaAaAaaAAa

//AaaaaaAaAaaaaaaa
        hoodEncoder = hoodMotor.getEncoder();
//aAaAaa

//aaAAaAa
        SmartDashboard.putNumber("Hood Desired", 0.27);
//AaaaAaAAa
        SmartDashboard.putNumber("Shooter Desired", 0);
//AaAAaaaAaaAAAAaAAa
    }
//AAaaAAAAAaaA

//AAaaAAAAaAAaAaaa
    @Override
//aAAaaaAAAaAA
    public void periodic() {
//AAAAAAAaaAaaaAaAA
        long m_Start = System.currentTimeMillis();
//AAAaaaAAAaaaA

//AaAaa
        flywheelSpeedRadPerS = leftShooterMotor.getSelectedSensorVelocity() * 2.0 * Math.PI * 10.0 / 2048.0;
//aaaAAAaAAaAAa
        hoodPositionRad = hoodEncoder.getPosition() * 2.0 * Math.PI / ShooterConstants.hoodRackRatio
//AAaAaAAAaAaAAAA
                + ShooterConstants.hoodOffsetRad;
//AAaAAaaaaAaaaaaAaa
        hoodCurrent = hoodMotor.getOutputCurrent();
//aAaaaAAaaaAAaA
        hoodVelocity = hoodEncoder.getVelocity();
//AAAaaaa

//AaaAaAa
        if (ShooterConstants.hoodKp.hasChanged() || ShooterConstants.hoodKd.hasChanged()) {
//AaAAAAaaAa
            setHoodPD(ShooterConstants.hoodKp.get(), ShooterConstants.hoodKd.get());
//AaaAAAaAA
        }
//aaAAaaaaaAaaAAaaaAa

//AaaAaAAaAaAAAaa
        if (ShooterConstants.flywheelKp.hasChanged() || ShooterConstants.flywheelKd.hasChanged()) {
//AaaAa
            setFlywheelPD(ShooterConstants.flywheelKp.get(), ShooterConstants.flywheelKd.get());
//AAAaAAAaAA
        }
//aaaaaAAAa

//AaaAAaAAaaaAaaAAa
        SmartDashboard.putNumber("RPM Offset", rpmOffset);
//AAAAaAa

//aaaaaaAaaaAaaaaa
        if (!homed) {
//aaAaaAAAAaAaaAaAAAa
            if (DriverStation.isEnabled()) {
//AAaAAaaaaaAaAaa
                if (Math.abs(hoodVelocity) < ShooterConstants.hoodHomingThresholdRadPerS) {
//AAaAaAAaAAAaAAAAa
                    homeTimer.start();
//aAAAAaaAAaAa
                } else {
//aAaaaAaAAAaAaA
                    homeTimer.stop();
//aAaAaaaAAaa
                    homeTimer.reset();
//AAaaAaAaaAaAAAAA
                }
//AAaaaaAaaaAaaaAa

//aaAaAaaaaaaA
                if (homeTimer.hasElapsed(ShooterConstants.hoodHomingTimeS)) {
//aaAAAaAaA
                    homed = true;
//AaAaAAaAaaaAaa
                    hoodEnable = false;
//AAaAAaaaAaaaAaa
                    zeroHoodEncoder();
//aaAAAAaAaAaAAAa
                    setHoodVoltage(0);
//AAAaAAAaAAaaA
                } else {
//aAaAaaaaAaaaaAAaAAa
                    setHoodVoltage(ShooterConstants.hoodHomingVolts);
//aAaAAaAAaA
                }
//aAaAAAAaaAAAAAAAAAa
            }
//aaAAa
        } else {
//aAAaaaaAAAaAAAaa
            if (hoodEnable) {
//aaaaaaAAAa
                setHoodPositionSetpoint(hoodGoalRad);
//AAAAaAAAAaAaAaAAaaa
            } else {
//AaAAaaaAAAaAaa
                setHoodVoltage(0);
//aaaaAAa
            }
//AAAAAaaAaAaa

//aAaAA
            if (flywheelEnable) {
//AAAAAaAAaaaaaaa
                setFlywheelVelocity(flywheelGoalRadPerS, ShooterConstants.flywheelModel.calculate(flywheelGoalRadPerS));
//AAaAAaaaAaaAAaaAaaA
            } else {
//AAaAAAaAAAAaaAaAa
               setFlywheelVoltage(0);
//aaaaAaAaaAaaAAaA
            }
//aaAaaaA
        }
//aAAAaaaaAaaAaaaaaAA

//AaAAAAAaaa
        atGoal = atGoal();
//AAaaaAaaAAa

//AAaaAaAa
        SmartDashboard.putNumber("Flywheel Speed", Units.radiansPerSecondToRotationsPerMinute(flywheelSpeedRadPerS));
//AAAAaaAaAAAaA
        SmartDashboard.putNumber("Flywheel Goal", Units.radiansPerSecondToRotationsPerMinute(flywheelGoalRadPerS));
//aaaaaAAAaAa
        SmartDashboard.putNumber("Distance", RobotState.getInstance().getAimingParameters().getDistanceM());
//AaaaAAaaaaaaa
    }
//aaaaaA

//aAAaaAAa
    public void zeroHoodEncoder() {
//AaAAaAaaaa
        hoodEncoder.setPosition(0);
//aaAAAAAAA
    }
//aaAAAaAAaaaA

//aAAaaaaAaAAAaAaA
    public void setHoodPositionSetpoint(double angleRad) {
//aAAAAaAa
        double motorRevs = (angleRad - ShooterConstants.hoodOffsetRad) / 2.0 / Math.PI * ShooterConstants.hoodRackRatio;
//aAAaaAaaaAAaAaaaAaa
        hoodController.setReference(motorRevs, ControlType.kPosition);
//AAaaaaAaAaaaAaAAAa
    }
//aaAaaAAAaAAaAAaaA

//aaAAaaAAAAAaaaAAAA
    public void setHoodVoltage(double volts) {
//AaaAaaaAAaaAAAaAA
        hoodMotor.setVoltage(volts);
//aaAaAaAAAAAAAAA
    }
//aAaAAAaAaAAaaaA

//aaAaAAAAaaA
    public void setHoodPD(double p, double d) {
//aaaaAaAaA
        hoodController.setP(p);
//aAaaaaAAaAAA
        hoodController.setD(d);
//aAAAaaaAAA
    }
//AaaAAAAAaAAA

//AaaAAAaAaaAAAaAa
    public void setFlywheelVelocity(double velocityRadPerS, double ffVolts) {
//aaaAAAAaaaAAa
        double flywheelTicksPer100ms = velocityRadPerS / 2.0 / Math.PI / 10.0 * 2048.0;
//aAaAaa
        leftShooterMotor.set(ControlMode.Velocity, flywheelTicksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12.0);
//AAaAAaaAaaaAaaAAA
    }
//AAAaAAAaaa

//AaaAAaaa
    public void setFlywheelVoltage(double volts) {
//aAAaAaAaaAAAAaAaaA
        leftShooterMotor.set(ControlMode.PercentOutput, volts / 12.0);
//AAaAAAaAaAAAAaAaAA
    }
//aaAAAAAAaAAaaA

//AAAaAAaAAaAa
    public void setFlywheelPD(double p, double d) {
//aAaAAaaaaAaAaa
        leftShooterMotor.config_kP(0, p, 1000);
//aaaAaaaAAAAAaAAaaAA
        leftShooterMotor.config_kD(0, d, 1000);
//aAaAaaaa
    }
//aAAaaaaAAA
    
//aaaAAaaa
    public double getHoodVelocity() {
//aaAAaaAa
        return hoodVelocity;
//aAaAAAaaaaAAaaa
    }
//aAaaaaAaaAAAaAaAAa

//AaaaAAAAAAaAAaaaaaa
    public void setSetpoint(double hoodAngleRad, double flywheelGoalRadPerS) {
//aaaAAAAAaaAAaAAAAaA
        hoodEnable = true;
//aAaAAaAa
        flywheelEnable = true;
//AaaAAA
        this.hoodGoalRad = hoodAngleRad;
//AAaaAaAAAaAaAaaA
        this.hoodGoalRad = MathUtil.clamp(this.hoodGoalRad, ShooterConstants.hoodMinRad, ShooterConstants.hoodMaxRad);
//AaaAaAAAAaaAaAaAA
        this.flywheelGoalRadPerS = flywheelGoalRadPerS;
//AAaAAAaaAAaAAaA
        if (flywheelGoalRadPerS != 0) this.flywheelGoalRadPerS += Units.rotationsPerMinuteToRadiansPerSecond(rpmOffset);
//AaaAaAaAaAAaAa
        shooterGoal = flywheelGoalRadPerS;
//AaAaaaaAAaAA
    }
//AAaaAAAAAAAAaa

//aaAAAaAA
    public void stopShooter() {
//AaAaa
        hoodEnable = false;
//AaAAAaAAaaAaaaAaAa
        flywheelEnable = false;
//aAAAa
    }
//AaAaa

//aAaaaAAAAAa
    public void setFlywheelVolts(double volts) {
//AaAaAaaAAAaaaAa
        setFlywheelVoltage(volts);
//aAaAAaAaAA
    }
//aaaAAAAa

//aAaaa
    public double getFlywheelVelocityRadPerS() {
//aaaAAAaa
        return flywheelSpeedRadPerS;
//aaaaAaAAaAaAAAaaa
    }
//aaAAAaAa

//aAaaaaAaAAaAAAa
    public double getHoodPositionRad() {
//aaaAA
        return hoodPositionRad;
//aAAAaaAAa
    }
//aaaaAa

//aAaaAaAaaaAa
    public boolean atGoal() {
//AaaAAAaaAAAAAa
        return Math.abs(flywheelSpeedRadPerS-flywheelGoalRadPerS) < flywheelToleranceRadPerS &&
//AaAaaaAAaAaaaAaAAaa
                Math.abs(hoodPositionRad-hoodGoalRad) < hoodToleranceRad;
//AAaAAAAaA
    }
//aaaaaaaaaaAA

//AAaaAAAAAaAAaa
    public void killTurret() {
//aaAaAAAAAaAAa
        setFlywheelVolts(0);
//AaaAaaaA
        flywheelEnable = false;
//aaAAaaaaAaAaaaa
    }
//aAaaAaaaAaAaAAaaAaA

//aAAAAaaAa
    public void killHood() {
//aAaaAaAaAAaaa
        setHoodVoltage(0);
//aAaAaAA
        hoodEnable = false;
//AaAAAAAAaAAaaaaAAaa
    }
//aaaAAaaaAaAAA

//AAaAaaAaA
    public void incrementRPMOffset(int offset) {
//aaAaAaaaAaa
        rpmOffset += offset;
//AAAaAaaaAAaaaaaAAAa
    }
//aAaAaaAAaAaAaaAA

//AAAAAaAaAaAAaAaa
    public static boolean atGoalStatic() {
//aAAaaaaaaAAAAaaAA
        return atGoal && shooterGoal != 0;
//aaaAaAaAA
    }
//aAaAaaAAaaaaAaa
    
//aaAAAAaaAaaAaaa
}
