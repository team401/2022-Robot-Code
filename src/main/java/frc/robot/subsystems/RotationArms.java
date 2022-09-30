package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants.DIOChannels;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

public class RotationArms extends SubsystemBase {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final DutyCycleEncoder leftEncoder;
    private final DutyCycleEncoder rightEncoder;


    public double leftPositionRad;
    public double rightPositionRad;
    public double leftCurrent;
    public double rightCurrent;

    public double leftVelocityRadPerS;
    public double rightVelocityRadPerS;

    // Speed and acceleration for regular moves
    private final TrapezoidProfile.Constraints normalConstraints = new TrapezoidProfile.Constraints(2 * Math.PI, 20);
    // Speed and acceleration for slower climb moves
    private final TrapezoidProfile.Constraints climbConstraints = new TrapezoidProfile.Constraints(1 * Math.PI , 5);

    private final ProfiledPIDController leftController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(), 
        normalConstraints);

    private final ProfiledPIDController rightController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(),
        normalConstraints);

    private final ProfiledPIDController leftClimbController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(), 
        climbConstraints);

    private final ProfiledPIDController rightClimbController = new ProfiledPIDController(
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(),
        climbConstraints);

    private boolean killed = false;
    private boolean homed = false;
    private boolean hasReset = false;

    private boolean leftOverride = false;
    private boolean rightOverride = false;

    private double leftLastPositionRad = 0;
    private double rightLastPositionRad = 0;

    private Timer homeTimer = new Timer();
    private Timer homeOverrideTimer = new Timer();
    private boolean homeOverrideTimerStarted = false;

    private boolean atGoalOverride = false;

    public RotationArms() {

        leftMotor = new CANSparkMax(CANDevices.leftRotationMotorID, MotorType.kBrushed);
        rightMotor = new CANSparkMax(CANDevices.rightRotationMotorID, MotorType.kBrushed);

        leftEncoder = new DutyCycleEncoder(DIOChannels.leftRotationArmEncoder);
        rightEncoder = new DutyCycleEncoder(DIOChannels.rightRotationArmEncoder);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        rightEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

        leftMotor.setSmartCurrentLimit(30);
        rightMotor.setSmartCurrentLimit(30);


        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        //leftMotor.burnFlash();
        //rightMotor.burnFlash();

        leftController.setTolerance(ClimberConstants.rotationPositionToleranceRad);
        rightController.setTolerance(ClimberConstants.rotationPositionToleranceRad);
        
        leftController.setConstraints(normalConstraints);
        rightController.setConstraints(normalConstraints);
        
        homeOverrideTimer.reset();
        homeOverrideTimer.stop();
        
    }

    @Override
    public void periodic() {
        
        long m_Start = System.currentTimeMillis();

        leftPositionRad = leftEncoder.get() * 2.0 * Math.PI - ClimberConstants.leftRotationOffset;
        rightPositionRad = rightEncoder.get() * 2.0 * Math.PI - ClimberConstants.rightRotationOffset;
        leftCurrent = leftMotor.getOutputCurrent();
        rightCurrent = rightMotor.getOutputCurrent();

        /*
        If the change in position between when home started and home finished (before resetting) is greater than (pi/2?) rad,
        then we know that the mechanical offset has not worked and it has gone all the way back.

        If rotation arm haven't homed correctly try and fix it, if we can't then fix it
        just send it positive velocity for n seconds to get them into the intake position, then kill it
        */

        if (!hasReset) {
            resetEncoder();
            hasReset = true;
        }

        double leftVelocityRadPerS = (leftPositionRad -  leftLastPositionRad) / 0.02;
        double rightVelocityRadPerS = (rightPositionRad -  rightLastPositionRad) / 0.02;
        
        leftLastPositionRad = leftPositionRad;
        rightLastPositionRad = rightPositionRad;

        //SmartDashboard.putNumber("leftVelocity", leftVelocityRadPerS);
        //SmartDashboard.putNumber("rightVelocity", rightVelocityRadPerS);

        if (ClimberConstants.rotationArmKp.hasChanged() || ClimberConstants.rotationArmKd.hasChanged()) {
            leftController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
            rightController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
            leftClimbController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
            rightClimbController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
        
        }

        // Positions with modulus to be between -pi, pi.  It is important to use these in the controllers
        // to prevent the arms from trying to go through their hard stops!
        double leftMod = MathUtil.angleModulus(leftPositionRad);
        double rightMod = MathUtil.angleModulus(rightPositionRad);

        // Reset PID controllers if the robot is not enabled
        if (DriverStation.isDisabled()) {
            leftController.reset(leftMod);
            rightController.reset(rightMod);

            leftController.setGoal(ClimberConstants.stowPositionRad);
            rightController.setGoal(ClimberConstants.stowPositionRad);
        }

        if (!homed) {
            if (DriverStation.isEnabled()) {
                /*if (!homeOverrideTimerStarted) {
                    homeOverrideTimer.reset();
                    homeOverrideTimer.start();
                    homeOverrideTimerStarted = true;
                }*/
                if (Math.abs(leftVelocityRadPerS) < ClimberConstants.rotationHomingThresholdRadPerS
                        && Math.abs(rightVelocityRadPerS) < ClimberConstants.rotationHomingThresholdRadPerS) {
                    homeTimer.start();
                } else {
                    homeTimer.stop();
                    homeTimer.reset();
                }

                if (homeTimer.hasElapsed(ClimberConstants.homingTimeS)) {
                    homed = true;

                    resetEncoder();
                    setLeftVolts(0);
                    setRightVolts(0);
                   
                    leftController.reset(leftPositionRad);
                    rightController.reset(rightPositionRad);

                } else {
                    setLeftVolts(ClimberConstants.rotationHomingVolts);
                    setRightVolts(ClimberConstants.rotationHomingVolts);
                }

                /*if (homeOverrideTimer.get() > 0.1) {
                    homed = true;

                    resetEncoder();
                    setLeftVolts(0);
                    setRightVolts(0);
                   
                    leftController.reset(leftPositionRad);
                    rightController.reset(rightPositionRad);

                    homeOverrideTimer.reset();
                    homeOverrideTimer.stop();

                }*/

            }
        } else {
            if (!killed) {
                double leftOutput = leftController.calculate(leftMod);
    
                double rightOutput = rightController.calculate(rightMod);
    
                if (!leftOverride)
                    setLeftVolts(leftOutput);
                if (!rightOverride)
                    setRightVolts(rightOutput);
    
      
            } else {
                if (!leftOverride)
                    setLeftVolts(0);
                if (!rightOverride)
                    setRightVolts(0);
            }

        }

        SmartDashboard.putBoolean("Rotation At Goal", atGoal());
    }

    public void setLeftVolts(double volts) {
        leftMotor.setVoltage(volts);
    }

    public void setRightVolts(double volts) {
        rightMotor.setVoltage(volts);
    }

    public void setLeftPercent(double percent) {
        setLeftVolts(percent * 12);
    }

    public void setRightPercent(double percent) {
        setRightVolts(percent * 12);
    }

    public void setDesiredPosition(double positionRad) {
        leftController.setConstraints(normalConstraints);
        rightController.setConstraints(normalConstraints);
        leftController.setGoal(positionRad);
        rightController.setGoal(positionRad);
    }

    public void setDesiredPositionSlow(double positionRad) {
        leftController.setConstraints(climbConstraints);
        rightController.setConstraints(climbConstraints);
        leftController.setGoal(positionRad);
        rightController.setGoal(positionRad);
    }

    public boolean atGoal() {
        return (leftController.atGoal() && rightController.atGoal()) || atGoalOverride;
    }

    public void resetEncoder() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void kill() {
        setLeftPercent(0);
        setRightPercent(0);
        killed = true;
    }

    public boolean getKilled() {
        return killed;
    }

    public double getGoal() {
        return leftController.getGoal().position;
    }

    public void home() {
        homed = false;
        homeTimer.reset();
        homeTimer.start();
    }

    public void setZero() {
        resetEncoder();
        setLeftVolts(0);
        setRightVolts(0);
       
        leftController.reset(leftPositionRad);
        rightController.reset(rightPositionRad);

        leftOverride = false;
        rightOverride = false;
    }

    public void overrideLeftPercent(double percent) {
        setLeftPercent(percent);
        leftOverride = true;
    }

    public void overrideRightPercent(double percent) {
        setRightPercent(percent);
        rightOverride = true;
    }

    public void stop() {
        setDesiredPosition(leftPositionRad);
    }

    public void setGoalOverride(boolean override) {
        atGoalOverride = override;
    }

    // Commands
    public final Command waitForMove() { return new WaitUntilCommand(this::atGoal); }
    public final Command moveToStow () { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.stowPositionRad), this); }
    public final Command moveToClimbGrab() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.climbGrabPositionRad), this); }
    public final Command moveToIntake() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.intakePositionRad), this); }
    public final Command moveToClimbSwing() { return new InstantCommand(() -> setDesiredPositionSlow(ClimberConstants.climbSwingPositionRad), this); }
    public final Command latchRotation() { return new InstantCommand(() -> setDesiredPositionSlow(ClimberConstants.rotationLatchRad), this); }
    // Maybe add another command for after swinging out to move them slightly back while telescoping down?
    
}