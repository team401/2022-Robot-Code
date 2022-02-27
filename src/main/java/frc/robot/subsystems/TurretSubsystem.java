package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.commands.superstructure.turret.limelight.BasicSearch;
import frc.robot.commands.superstructure.turret.limelight.Tracking;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * This link was a helpful example:
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/MagEncoder_Absolute/src/main/java/frc/robot/Robot.java
 */

public class TurretSubsystem extends SubsystemBase{

    private final WPI_TalonFX turretMotor = new WPI_TalonFX(CANDevices.turretMotorID);
    private CANCoderConfiguration turretEncoderSettings = new CANCoderConfiguration();

    private final TalonFXConfiguration turretMotorSettings = new TalonFXConfiguration();
    private final CANCoder turretMagEncoder = new CANCoder(CANDevices.turretEncoderID);

    // works for the best in .1 rad increments
    private double kP = 1.2;
    private double kI = 0.5;
    private double kD = 0;

    private final PIDController turretController = new PIDController(kP, kI, kD);

    public TurretSubsystem() {
       
        //Configure Encoder Settings 
        turretMagEncoder.configFactoryDefault();

            turretEncoderSettings.unitString = "rad";
            turretEncoderSettings.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
            turretEncoderSettings.sensorDirection = false;

            turretEncoderSettings.initializationStrategy = 
                SensorInitializationStrategy.BootToAbsolutePosition;

            turretEncoderSettings.magnetOffsetDegrees = 
                SuperstructureConstants.turretMagEncoderOffsetDegrees;

        turretMagEncoder.configAllSettings(turretEncoderSettings);

        //Configure Turret Settings
        turretMotor.configFactoryDefault();
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

            turretMotorSettings.initializationStrategy = SensorInitializationStrategy.BootToZero;
            turretMotorSettings.forwardSoftLimitEnable = true;
            turretMotorSettings.reverseSoftLimitEnable = true;
            turretMotorSettings.slot0.kP = kP;
            turretMotorSettings.slot0.kI = kI;
            turretMotorSettings.slot0.kD = kD;

        turretMotor.configAllSettings(turretMotorSettings);

        //Set Turret Integrated Encoder to Aboslute Encoder Position
        turretMotor.setSelectedSensorPosition(convertTurretDegreesToTicks(
            turretMagEncoder.getAbsolutePosition()));
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Turret Position Radians", getTurretPositionRadians());
        SmartDashboard.putNumber("Turret Position Ticks", getEncoderPositionTicks());    
    }

    public static double convertTurretTicksToRadians(double ticks) {
        return ticks / SuperstructureConstants.turretEncoderCountsPerRevolution 
                / SuperstructureConstants.turretGearReduction * (2 * Math.PI);
    } 

    public static double convertTurretRadiansToTicks(double radians) {
        return radians / (2 * Math.PI) * SuperstructureConstants.turretEncoderCountsPerRevolution
                * SuperstructureConstants.turretGearReduction;
    } 

    public static double convertTurretDegreesToTicks(double degrees) {
        return degrees / 360 * SuperstructureConstants.turretEncoderCountsPerRevolution
                * SuperstructureConstants.turretGearReduction;
    } 

    public double getTurretPositionRadians() {

        return getEncoderPositionTicks() / SuperstructureConstants.turretEncoderCountsPerRevolution 
                / SuperstructureConstants.turretGearReduction * (2*Math.PI);

    }

    public double getEncoderPositionTicks() {

        return turretMotor.getSelectedSensorPosition();
    }

    public double getTurretVelocityRadPerSec() {

        return turretMotor.getSelectedSensorVelocity() / SuperstructureConstants.turretEncoderCountsPerRevolution
                * (2 * Math.PI) * 10 /  SuperstructureConstants.turretGearReduction;

    }

    public void runTurretPercent(double percent) {

        turretMotor.set(ControlMode.PercentOutput, percent);
        
    }

    public void runTurretVoltage(double volts) {

        turretMotor.setVoltage(volts);

    }

    public void setTurretDesiredClosedState(double desiredPositionRad) {

        double output = turretController.calculate(getTurretPositionRadians(), desiredPositionRad);
        runTurretPercent(output);

    }

    public boolean isWithinEdges() {

        return Math.abs(getTurretPositionRadians()) < SuperstructureConstants.rightTurretExtremaRadians;

    }

    public void setTurretEncoderRightSoftLimit(double positionTicks) {

        turretMotor.setSelectedSensorPosition(positionTicks);

    }

    public void setTurretEncoderRightExtrema() {

        turretMotor.setSelectedSensorPosition(convertTurretRadiansToTicks(SuperstructureConstants.rightTurretExtremaRadians));

        //turretMotor.configForwardSoftLimitThreshold(convertTurretRadiansToTicks(SuperstructureConstants.rightTurretExtremaRadians));
        //turretMotor.configReverseSoftLimitThreshold(convertTurretRadiansToTicks(SuperstructureConstants.leftTurretExtremaRadians));

    }

}
