package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.commands.superstructure.turret.limelight.BasicSearch;
import frc.robot.commands.superstructure.turret.limelight.Tracking;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * This link was a helpful example:
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/MagEncoder_Absolute/src/main/java/frc/robot/Robot.java
 */

public class TurretSubsystem extends SubsystemBase{

    private final WPI_TalonFX turretMotor = new WPI_TalonFX(0);

    // works for the best in .1 rad increments
    private double kP = 1.2;
    private double kI = 0.5;
    private double kD = 0;

    private final PIDController turretController = new PIDController(kP, kI, kD);

    //private double centerOffsetTicks = 0;

    //private boolean shouldBeTracking = false;

    public TurretSubsystem() {

        turretMotor.configFactoryDefault();
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

        turretMotor.setSelectedSensorPosition(0, 0, 10);

        //centerOffsetTicks = getEncoderPositionTicks();

        turretMotor.configForwardSoftLimitEnable(true);
        turretMotor.configReverseSoftLimitEnable(true);
        //turretMotor.configForwardSoftLimitThreshold(SuperstructureConstants.rightTurretSoftLimitTicks);
        //turretMotor.configReverseSoftLimitThreshold(SuperstructureConstants.leftTurretSoftLimitTicks);

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Turret Position Radians", getTurretPositionRadians());
        SmartDashboard.putNumber("Turret Position Ticks", getEncoderPositionTicks());
        SmartDashboard.putBoolean("isWithinEdges", isWithinEdges());    
    }

    public static double convertTurretTicksToRadians(double ticks) {
        return ticks / SuperstructureConstants.turretEncoderCountsPerRevolution 
                / SuperstructureConstants.turretGearReduction * (2*Math.PI);
    } 

    public double getTurretPositionRadians() {

        return getEncoderPositionTicks() / SuperstructureConstants.turretEncoderCountsPerRevolution 
                / SuperstructureConstants.turretGearReduction * (2*Math.PI);

    }

    public double getEncoderPositionTicks() {

        return turretMotor.getSelectedSensorPosition();
    }

    /*public double getTurretPositionPulseWidth() {
        
        return turretMotor.getSensorCollection().getPulseWidthPosition();

    }*/

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

    /*public void startTracking(LimelightSubsystem limelight, TurretSubsystem turret) {

        isTracking(true);

        new BasicSearch(limelight, turret).schedule();;
        
    }

    public void isTracking(boolean shouldbe) {

        shouldBeTracking = shouldbe;

    }

    public boolean shouldBeTracking() { 

        return shouldBeTracking;

    }*/

}