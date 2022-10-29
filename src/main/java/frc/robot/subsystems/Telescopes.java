package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.playingwithfusion.TimeOfFlight;

public class Telescopes extends SubsystemBase {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final TimeOfFlight leftLidar;
    private final TimeOfFlight rightLidar;

    public double leftLidarPositionM;
    public double rightLidarPositionM;
    public double leftEncoderPositionM;
    public double rightEncoderPositionM;
    public double leftVelocityMPerS;
    public double rightVelocityMPerS;
    public double leftCurrent;
    public double rightCurrent;

    private final ProfiledPIDController leftController = new ProfiledPIDController(
        ClimberConstants.telescopeArmKp, 0, ClimberConstants.telescopeArmKd,
        new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocityM, ClimberConstants.telescopeAccelerationM)
    );

    private final ProfiledPIDController rightController = new ProfiledPIDController(
        ClimberConstants.telescopeArmKp, 0, ClimberConstants.telescopeArmKd,
        new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocityM, ClimberConstants.telescopeAccelerationM)
    );

    private final double deltaTime = 0.02; // 20ms between each cycle

    private double desiredPositionM = 0; // position the telescopes attempt to reach in periodic

    private boolean motorOverride = false; // true = periodic doesn't set motor voltage/percent
    private boolean atGoalOverride = false; // true = atGoal() always returns true

    private boolean leftHomed = false;
    private boolean rightHomed = false;

    private final Timer leftHomeTimer = new Timer();
    private final Timer rightHomeTimer = new Timer();

    public Telescopes() {

        // Initialization
        leftMotor = new CANSparkMax(CANDevices.leftTelescopingMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(CANDevices.rightTelescopingMotorID, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftLidar = new TimeOfFlight(CANDevices.leftLidar);
        rightLidar = new TimeOfFlight(CANDevices.rightLidar);

        // Options
        leftMotor.setSmartCurrentLimit(80);
        rightMotor.setSmartCurrentLimit(80);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        
        leftEncoder.setPositionConversionFactor(ClimberConstants.telescopeConversionFactor);
        rightEncoder.setPositionConversionFactor(ClimberConstants.telescopeConversionFactor);
        leftEncoder.setVelocityConversionFactor(ClimberConstants.telescopeConversionFactor);
        rightEncoder.setVelocityConversionFactor(ClimberConstants.telescopeConversionFactor);

        leftLidar.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        rightLidar.setRangingMode(TimeOfFlight.RangingMode.Short, 24);

        leftController.setGoal(ClimberConstants.telescopeHomePositionM);
        rightController.setGoal(ClimberConstants.telescopeHomePositionM);

        leftController.setTolerance(ClimberConstants.telescopeGoalToleranceM);
        rightController.setTolerance(ClimberConstants.telescopeGoalToleranceM);

        leftHomeTimer.start();
        rightHomeTimer.start();

    }

    @Override
    public void periodic() {

        if (leftLidar.getStatus() == TimeOfFlight.Status.Valid)
            leftLidarPositionM = leftLidar.getRange() - ClimberConstants.telescopeOffsetM;
        if (rightLidar.getStatus() == TimeOfFlight.Status.Valid)
            rightLidarPositionM = rightLidar.getRange() - ClimberConstants.telescopeOffsetM;

        leftEncoderPositionM = leftEncoder.getPosition();
        rightEncoderPositionM = rightEncoder.getPosition();

        leftVelocityMPerS = leftEncoder.getVelocity() / 60.0;
        rightVelocityMPerS = rightEncoder.getVelocity() / 60.0;

        leftCurrent = leftMotor.getOutputCurrent();
        rightCurrent = rightMotor.getOutputCurrent();

        long m_Start = System.currentTimeMillis();

        if (!DriverStation.isEnabled()) {
            if (isLeftLidarValid())
                leftController.reset(leftLidarPositionM);
            else
                leftController.reset(leftEncoderPositionM);
            if (isRightLidarValid())
                rightController.reset(rightLidarPositionM);
            else
                rightController.reset(rightEncoderPositionM);
            
            leftHomeTimer.reset();
            rightHomeTimer.reset();
        }

        if (DriverStation.isEnabled() && !motorOverride) {
            // Left
            if (isLeftLidarValid()) {
                double output = leftController.calculate(leftLidarPositionM, desiredPositionM);
                output -= output < 0 ? 0.7 : 0; // FF
                setLeftVolts(output);

                leftHomed = true;
                if (Math.abs(leftLidarPositionM - leftEncoderPositionM) > ClimberConstants.telescopeGoalToleranceM)
                    setLeftEncoder(leftLidarPositionM);
            }
            else {
                if (leftHomed) {
                    double output = leftController.calculate(leftEncoderPositionM, desiredPositionM);
                    output -= output < 0 ? 0.7 : 0; // FF
                    setLeftVolts(output);
                }
                else {
                    if (Math.abs(leftVelocityMPerS) > ClimberConstants.telescopeHomingThresholdMPerS) {
                        leftHomeTimer.reset();
                    }
                    else if (leftHomeTimer.hasElapsed(ClimberConstants.homingTimeS)){
                        leftHomed = true;
                        setLeftEncoder(0);
                        setLeftVolts(0);
                        leftController.reset(leftEncoderPositionM);
                    }
                    else {
                        setLeftVolts(ClimberConstants.telescopeHomingVolts);
                    }
                }
            }

            
            // Right
            if (isRightLidarValid()) {
                double output = rightController.calculate(rightLidarPositionM, desiredPositionM);
                output -= output < 0 ? 0.7 : 0; // FF
                setRightVolts(output);

                rightHomed = true;
                if (Math.abs(rightLidarPositionM - rightEncoderPositionM) > ClimberConstants.telescopeGoalToleranceM)
                    setRightEncoder(rightLidarPositionM);
            }
            else {
                if (rightHomed) {
                    double output = rightController.calculate(rightEncoderPositionM, desiredPositionM);
                    output -= output < 0 ? 0.7 : 0; // FF
                    SmartDashboard.putNumber("Right Output", output);
                    setRightVolts(output);
                }
                else {
                    if (Math.abs(rightVelocityMPerS) > ClimberConstants.telescopeHomingThresholdMPerS) {
                        rightHomeTimer.reset();
                    }
                    else if (rightHomeTimer.hasElapsed(ClimberConstants.homingTimeS)){
                        rightHomed = true;
                        setRightEncoder(0);
                        setRightVolts(0);
                        rightController.reset(rightEncoderPositionM);
                    }
                    else {
                        setRightVolts(ClimberConstants.telescopeHomingVolts);
                    }
                }
            }
        }

        SmartDashboard.putBoolean("Telescopes At Goal", atGoal());
        SmartDashboard.putNumber("Left Homed", leftHomed ? 1 : 0);
        SmartDashboard.putNumber("Right Homed", rightHomed ? 1 : 0);
        SmartDashboard.putNumber("Right Pos M", rightEncoderPositionM);
        SmartDashboard.putNumber("Left Pos M", leftEncoderPositionM);
        SmartDashboard.putNumber("Desired Right Pos M", desiredPositionM);

        SmartDashboard.putNumber("Left Draw", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Draw", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Left vel", leftVelocityMPerS);
        SmartDashboard.putNumber("Right vel", rightVelocityMPerS);


    }

    public void setLeftEncoder(double positionM) {
        leftEncoder.setPosition(positionM);
    }

    public void setRightEncoder(double positionM) {
        rightEncoder.setPosition(positionM);
    }

    public void setLeftVolts(double volts) {
        leftMotor.setVoltage(volts);        
    }

    public void setRightVolts(double volts) {
        rightMotor.setVoltage(volts);        
    }

    public double getLeftCurrentDraw() {
        return leftMotor.getOutputCurrent();
    }
    
    public double getRightCurrentDraw() {
        return rightMotor.getOutputCurrent();
    }

    public boolean isLeftLidarValid() {
        return leftLidar.getStatus() == TimeOfFlight.Status.Valid;
    }

    public boolean isRightLidarValid() {
        return rightLidar.getStatus() == TimeOfFlight.Status.Valid;
    }

    public void setLeftPercent(double percent) {
        setLeftVolts(percent * 12);
    }

    public void setRightPercent(double percent) {
        setRightVolts(percent * 12);
    }

    public void setDesiredPosition(double positionM) {
        desiredPositionM = positionM;
    }

    public void jogUp() {
        desiredPositionM += deltaTime * ClimberConstants.telescopeCruiseVelocityM;
        if (desiredPositionM > ClimberConstants.telescopeMaxPositionM)
            desiredPositionM = ClimberConstants.telescopeMaxPositionM;
    }

    public void jogDown() {
        desiredPositionM -= deltaTime * ClimberConstants.telescopeCruiseVelocityM;
        if (desiredPositionM < ClimberConstants.telescopeHomePositionM)
            desiredPositionM = ClimberConstants.telescopeHomePositionM;
    }

    public boolean atGoal() {
        return atGoalOverride || (leftController.atGoal() && rightController.atGoal());
    }

    public boolean passedRotationSafePosition() {
        double leftPositionM = isLeftLidarValid() ? leftLidarPositionM : leftEncoderPositionM;
        double rightPositionM = isRightLidarValid() ? rightLidarPositionM : rightEncoderPositionM;
        return leftPositionM <= ClimberConstants.telescopeRotationSafePositionM &&
                rightPositionM <= ClimberConstants.telescopeRotationSafePositionM;
    }

    public void stop() {
        setDesiredPosition(isLeftLidarValid() ? leftLidarPositionM : leftEncoderPositionM);
    }

    public void setMotorOverride(boolean override) {
        motorOverride = override;
        setLeftVolts(0);
        setRightVolts(0);
    }

    public void setAtGoalOverride(boolean override) {
        atGoalOverride = override;
    }

    public void home() {
        leftHomed = false;
        rightHomed = false;
        leftHomeTimer.reset();
        rightHomeTimer.reset();
    }

    // Commands
    public final Command waitForMove() { return new WaitUntilCommand(this::atGoal); }
    public final Command waitForRotationSafePosition() { return new WaitUntilCommand(this::passedRotationSafePosition); }
    public final Command moveToPop() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePopAboveRungM), this); }
    public final Command moveToFull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeMaxPositionM), this); }
    public final Command moveToLatch() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeLatchM), this); }
    public final Command moveToPull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePullPositionM), this); }
    public final Command moveToSwing() {return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeSwingPositionM), this); }
    
}
