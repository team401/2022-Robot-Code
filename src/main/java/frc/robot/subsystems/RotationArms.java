//aaaAaA
package frc.robot.subsystems;
//aaaAAAAaaaa

//aAaAaaaaaaaAaaaAa
import edu.wpi.first.wpilibj2.command.Command;
//aaaAAAaAa
import edu.wpi.first.wpilibj2.command.InstantCommand;
//AaAAaAaAaAAa
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//aAAaaaAAAAaaaAAAAAa

//AAAAaAaAA
import edu.wpi.first.math.MathUtil;
//AaAaAaaAaAAaaaaa
import edu.wpi.first.math.controller.ProfiledPIDController;
//AaAaAa
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//aaaaaAAaAaaAAAaa
import edu.wpi.first.math.util.Units;
//AAAaAaAAaAaAaAaaaA
import edu.wpi.first.wpilibj.DriverStation;
//AaAaAaAaaaaAAAaaaAa
import edu.wpi.first.wpilibj.Timer;
//aAaAaaAAaAAaAAAaAa
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//AaaAaaaaaaaAAAa
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//aAaAAaaAaaAAAaaAAaa
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//aAAAaaaAaAaaaAAA

//AAaaA
import com.revrobotics.CANSparkMax;
//aaaaAA
import com.revrobotics.CANSparkMax.IdleMode;
//AaaaA
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//aaaAaa
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
//AAaaAAAAaaAaAAaAaaa

//AAaAAaAAAaAaa
import frc.robot.Constants.DIOChannels;
//aAaaAaAaAaaAAaAAa
import frc.robot.Constants.CANDevices;
//aaAAa
import frc.robot.Constants.ClimberConstants;
//aaAAaaaaAaaaaaAAa

//aaaaAAAA
public class RotationArms extends SubsystemBase {
//AaaAaaAaaAaAAAA

//AAaAAAaAaaAAAaAaaa
    private final CANSparkMax leftMotor;
//aAaaaaaaAAaAa
    private final CANSparkMax rightMotor;
//aAAAAAaAaAAa

//Aaaaaa
    private final DutyCycleEncoder leftEncoder;
//AaaAAaaaaaaaAA
    private final DutyCycleEncoder rightEncoder;
//AaAaaA

//AAAaaaaaAaAAAAAA

//AaaAA
    public double leftPositionRad;
//aaAaAaaAaAa
    public double rightPositionRad;
//AaAaaAa
    public double leftCurrent;
//aaaAaaAaAAA
    public double rightCurrent;
//AaAAAaAaaAAAaAa

//aAAaaAaaAaaAaaaAa
    public double leftVelocityRadPerS;
//aAaAaaAAA
    public double rightVelocityRadPerS;
//AaaaAAaaAaaAa

//aAaAaaa
    // Speed and acceleration for regular moves
//aaaaaaaaaAaaaaaaAa
    private final TrapezoidProfile.Constraints normalConstraints = new TrapezoidProfile.Constraints(2 * Math.PI, 20);
//AAAaAaaaaAAaAaa
    // Speed and acceleration for slower climb moves
//AAaAaaAAaA
    private final TrapezoidProfile.Constraints climbConstraints = new TrapezoidProfile.Constraints(1 * Math.PI , 5);
//aaAAaaaaaaaA

//aAAAa
    private final ProfiledPIDController leftController = new ProfiledPIDController(
//AAAaaAa
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(), 
//AAaaAaAaAaAAA
        normalConstraints);
//aAAAAaAaAaAaAA

//AAAAAaAAa
    private final ProfiledPIDController rightController = new ProfiledPIDController(
//aaaAaaaAAaaAaaAaa
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(),
//AaaAaAaaa
        normalConstraints);
//aAAAAaA

//aaAAAAaa
    private final ProfiledPIDController leftClimbController = new ProfiledPIDController(
//aaaAAa
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(), 
//AAaaaaAA
        climbConstraints);
//aaAaaAAAaaaaAa

//AAaaAaAAAaaaAaa
    private final ProfiledPIDController rightClimbController = new ProfiledPIDController(
//aAAAAAaaAAAAa
        ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get(),
//aAaaAAAaaa
        climbConstraints);
//aAAaAAaaAaa

//aaAaaaAAaAAAAAa
    private boolean killed = false;
//AAAaaaAaA
    private boolean homed = false;
//aAAaaAAaaaAAAAA
    private boolean hasReset = false;
//AAAaaaA

//AaAAaAA
    private boolean leftOverride = false;
//AAAaaAAaa
    private boolean rightOverride = false;
//aAAaaaaA

//aaAAaAaAAAaaAaaAAa
    private double leftLastPositionRad = 0;
//aaaaAaaAaAAaaAAaAaa
    private double rightLastPositionRad = 0;
//AAaAAA

//aAaAaaAAaaaa
    private Timer homeTimer = new Timer();
//aAAAaaA
    private Timer homeOverrideTimer = new Timer();
//AAaAa
    private boolean homeOverrideTimerStarted = false;
//aAAaaaAaAa

//AaaaAAaA
    private boolean atGoalOverride = false;
//AaAaAa

//aaAaAaaAAAAaaAAA
    public RotationArms() {
//AaAaAAaAaAAAAAA

//AaaaAaAAAAaAAa
        leftMotor = new CANSparkMax(CANDevices.leftRotationMotorID, MotorType.kBrushed);
//aaaaaAAaaaAAAA
        rightMotor = new CANSparkMax(CANDevices.rightRotationMotorID, MotorType.kBrushed);
//aaAaaaA

//aAaAaAAAaaaaAAAa
        leftEncoder = new DutyCycleEncoder(DIOChannels.leftRotationArmEncoder);
//AAaaAaAAaaAaAA
        rightEncoder = new DutyCycleEncoder(DIOChannels.rightRotationArmEncoder);
//AAAAaAaaaAAAAaAaa

//AAAAAAaAAAaaa
        leftMotor.restoreFactoryDefaults();
//aaAAaAaaaAaaAaAaa
        rightMotor.restoreFactoryDefaults();
//aAaaAAaA

//aAAAaaaaaAa
        leftMotor.setIdleMode(IdleMode.kBrake);
//aAAAAAaaaAaaaAAaaA
        rightMotor.setIdleMode(IdleMode.kBrake);
//aaaAa

//aaaAaaAaaaaaAAAAaAA
        leftEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
//aAaaAA
        rightEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
//aAAAaaaaaaAAaaaAaaa

//AAAAaaaaaAaaAaaaaA
        leftMotor.setSmartCurrentLimit(30);
//AAaAaAaAA
        rightMotor.setSmartCurrentLimit(30);
//AAaAAaAA

//aaAaAAaaA

//aaaAaAAaAAaAAAaAAAa
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
//AAaAaaaAAAa
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
//aAAAaaAAAaAaaa
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
//aaAAaaAAAAAA
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
//aaAaAAAaaaaAAaaAaA
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
//AaaaaAAaAaaAA
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
//AaaAAaAAa
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
//aaAAAAaaaAa
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
//AAAaAAAaAaaAAAAAA

//AAAAAAaaaaa
        leftMotor.setInverted(false);
//AaAAaAA
        rightMotor.setInverted(true);
//AAaAAaaA

//aaaAaAAaAAAaAA
        //leftMotor.burnFlash();
//aaAaaaAAaaa
        //rightMotor.burnFlash();
//aAaaAaaaaaAAAaaA

//AaAAaAAa
        leftController.setTolerance(ClimberConstants.rotationPositionToleranceRad);
//AAaaaaAaAaaa
        rightController.setTolerance(ClimberConstants.rotationPositionToleranceRad);
//AaaAAAaAa
        
//AaaAaAAaAaAAAAAaAA
        leftController.setConstraints(normalConstraints);
//aAAaAAAaAaAAa
        rightController.setConstraints(normalConstraints);
//aAAaaAAAaaaaAAAaAa
        
//AaAAaAAAaaaAaaAaAAa
        homeOverrideTimer.reset();
//AaaaAAaAa
        homeOverrideTimer.stop();
//AAaAaaAAA
        
//AaAAaaA
    }
//aAAaaAaaaA

//AaaaAaaAAaAAAa
    @Override
//aAAaAaaAaAa
    public void periodic() {
//aAaAaaAaAaAAaAAaA
        
//aAAAaaAAAaAa
        long m_Start = System.currentTimeMillis();
//aaAaAAAAaAAAAaAAaaA

//aaAaaa
        leftPositionRad = leftEncoder.get() * 2.0 * Math.PI - ClimberConstants.leftRotationOffset;
//aaAaaaaA
        rightPositionRad = rightEncoder.get() * 2.0 * Math.PI - ClimberConstants.rightRotationOffset;
//aAAAaa
        leftCurrent = leftMotor.getOutputCurrent();
//aAaaAaAAaaaaaAaAAAA
        rightCurrent = rightMotor.getOutputCurrent();
//aaaAaaaAAaa

//aAaaAaaAAaAaAAaAa
        /*
//aaaAAAaAAaAAaAa
        If the change in position between when home started and home finished (before resetting) is greater than (pi/2?) rad,
//AAAaAAaAaaaaAaA
        then we know that the mechanical offset has not worked and it has gone all the way back.
//AaAAaAaAaaAAAaaa

//AaAAAAAaAAAaAaAA
        If rotation arm haven't homed correctly try and fix it, if we can't then fix it
//aaaaA
        just send it positive velocity for n seconds to get them into the intake position, then kill it
//AAaAaAaaaAA
        */
//AaAaaaAaAAAA

//AAaAAaAaAAaaaAaa
        if (!hasReset) {
//AAAAAAAaaAaaAAaa
            resetEncoder();
//AAAaAaA
            hasReset = true;
//AaAaAaAaAAAa
        }
//aAaAAaaaaaaaaAaAa

//aAaAaaAAaaaaaaAAAA
        double leftVelocityRadPerS = (leftPositionRad -  leftLastPositionRad) / 0.02;
//aaAAaAAAAAaaa
        double rightVelocityRadPerS = (rightPositionRad -  rightLastPositionRad) / 0.02;
//AaaAaAAaaaAAaa
        
//AaaAaAAAAA
        leftLastPositionRad = leftPositionRad;
//AaaaAaAAAAAA
        rightLastPositionRad = rightPositionRad;
//AAAAAAaaaAaaaAaAAaa

//aAaAaAAaaaAAaaa
        //SmartDashboard.putNumber("leftVelocity", leftVelocityRadPerS);
//AAaAAAAaaAAaAAAA
        //SmartDashboard.putNumber("rightVelocity", rightVelocityRadPerS);
//aAAAaa

//AaaAaaaAaAAAAaaaa
        if (ClimberConstants.rotationArmKp.hasChanged() || ClimberConstants.rotationArmKd.hasChanged()) {
//AaaAa
            leftController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
//AAAaAaAAAA
            rightController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
//aAAaA
            leftClimbController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
//aaaaaaaAAaaaa
            rightClimbController.setPID(ClimberConstants.rotationArmKp.get(), 0, ClimberConstants.rotationArmKd.get());
//AAaAaAaAaaa
        
//AAaAAaA
        }
//aAaAaaAaaAA

//aaaaaaAaAaaaAaA
        // Positions with modulus to be between -pi, pi.  It is important to use these in the controllers
//aAAAaaaa
        // to prevent the arms from trying to go through their hard stops!
//aAaaaAaaAAAaaAaaAa
        double leftMod = MathUtil.angleModulus(leftPositionRad);
//AAaaAaaaaAaAaAa
        double rightMod = MathUtil.angleModulus(rightPositionRad);
//aAaaaaaaaAAAaaa

//aAAaaaaaaaaAAAAaaA
        // Reset PID controllers if the robot is not enabled
//AAAaaAaaaAaa
        if (DriverStation.isDisabled()) {
//AAaaaA
            leftController.reset(leftMod);
//aaaAAaAAAAaAAaaAaaa
            rightController.reset(rightMod);
//AAAaAAAaAaAaaAaa

//AAAaaaaAAAaaAa
            leftController.setGoal(ClimberConstants.stowPositionRad);
//aaAAaAaaAaAaaAA
            rightController.setGoal(ClimberConstants.stowPositionRad);
//AAAAAaAaAaa
        }
//aaaAaAaAAAAaaaaAA

//aAAaAaAAaAaaAA
        if (!homed) {
//AaaAaaaaAAAaaAaA
            if (DriverStation.isEnabled()) {
//AAaAAAAAAaaa
                /*if (!homeOverrideTimerStarted) {
//aaaaaaAAAAaa
                    homeOverrideTimer.reset();
//aaAAAAaaaAAaaAAaAaA
                    homeOverrideTimer.start();
//AAAaaaaaaAaaa
                    homeOverrideTimerStarted = true;
//AAAAAaAAaaaAaaa
                }*/
//AAAaAAaa
                if (Math.abs(leftVelocityRadPerS) < ClimberConstants.rotationHomingThresholdRadPerS
//AAaaaaAA
                        && Math.abs(rightVelocityRadPerS) < ClimberConstants.rotationHomingThresholdRadPerS) {
//AAAaAAaAaAAaaAAa
                    homeTimer.start();
//AaAaAAaaaaAAAAa
                } else {
//AAAaaAaaAaA
                    homeTimer.stop();
//aaAAAaaA
                    homeTimer.reset();
//AaaaAAaAAaA
                }
//aAAaaAAaAaAAaAAAA

//AaAaAaaAa
                if (homeTimer.hasElapsed(ClimberConstants.homingTimeS)) {
//aAAAaAaaAA
                    homed = true;
//aaaaaaAaaaAA

//aAAAAaAAaaa
                    resetEncoder();
//aAaAA
                    setLeftVolts(0);
//AaAaAAAaaAaAaaAAAa
                    setRightVolts(0);
//AAaaaAaaAaa
                   
//aaaaaAAaa
                    leftController.reset(leftPositionRad);
//AAaaaAaaaaAA
                    rightController.reset(rightPositionRad);
//aaAAaaAAAAAaaaa

//aAAaAAaAaAaaaAA
                } else {
//AaaaAAa
                    setLeftVolts(ClimberConstants.rotationHomingVolts);
//aAAaAaAAaaA
                    setRightVolts(ClimberConstants.rotationHomingVolts);
//AaAAaaAAaAAaaaA
                }
//AaAAAaAAaaAAaaaA

//aAAaaaaaAAaAAaAAAaa
                /*if (homeOverrideTimer.get() > 0.1) {
//AaAAaaaaaAAAaaAaaa
                    homed = true;
//aAAAAAaa

//AaaAAAAaaaAaAaAaaAA
                    resetEncoder();
//aaAAAAAaAaAAaA
                    setLeftVolts(0);
//AaAaaaaaa
                    setRightVolts(0);
//AAAaAaAaAAaA
                   
//AAaaaAaaaaAaa
                    leftController.reset(leftPositionRad);
//aaAAAA
                    rightController.reset(rightPositionRad);
//aaaaAAA

//AAaaAAAaaAaAaaaaaAA
                    homeOverrideTimer.reset();
//aAaAAAAaaA
                    homeOverrideTimer.stop();
//aaAaA

//aaaaaAaaaaaaaaaA
                }*/
//AaAaAaaaaAAAa

//aaaaAaAAAAaAaaAA
            }
//AAaaaAaaaaAaAa
        } else {
//aAAAaaa
            if (!killed) {
//AaAaAAAAAAaaaaaA
                double leftOutput = leftController.calculate(leftMod);
//aAAAAaaAa
    
//aaaAAAAaAaAAAAa
                double rightOutput = rightController.calculate(rightMod);
//AaAAaaaAAaaaaAa
    
//aAaaaAaaaAA
                if (!leftOverride)
//aAAaaAAAAAAAaAAAAA
                    setLeftVolts(leftOutput);
//aaAAaAAAaA
                if (!rightOverride)
//AaAAaAaaaAAAAAAaAaa
                    setRightVolts(rightOutput);
//aaaAaaaaaaa
    
//AAAaaAaAAaAa
      
//aaaaaAaaa
            } else {
//aAAaAAaAAAaaA
                if (!leftOverride)
//AaAAAAaAAAaAAAaaa
                    setLeftVolts(0);
//AAAaaAaaA
                if (!rightOverride)
//AAaAaAaaaaAaa
                    setRightVolts(0);
//AaaaAaaaAaaAaaa
            }
//AAaAA

//AaAaA
        }
//AaaAaAAAAAA

//aAaaAAAAAaaaAAAaaaA
        SmartDashboard.putBoolean("Rotation At Goal", atGoal());
//AaaaaaAaAAaaAaAAA
    }
//AAaAaaaAaA

//AaaaAaAaaaAaAaA
    public void setLeftVolts(double volts) {
//AAaAAAAAaAa
        leftMotor.setVoltage(volts);
//AAAaaAaAAAAAa
    }
//AAaAaaAAA

//AaAaAAaAAaaaaAAAAaA
    public void setRightVolts(double volts) {
//AAAAAaAaAAAaaAaaaAa
        rightMotor.setVoltage(volts);
//aAAaAaaaaAaAaAaAaaa
    }
//AAaaAaaAAaAAAA

//aAaAaA
    public void setLeftPercent(double percent) {
//aaaaAaaa
        setLeftVolts(percent * 12);
//aaaaAAaaAaAAaaaAaA
    }
//AaAaAAaAAaaA

//aaAAaaAaa
    public void setRightPercent(double percent) {
//aaAaaaAaAaa
        setRightVolts(percent * 12);
//aAaaaAAAAaaaaAAa
    }
//AaAaAAaaAAAaAaaA

//AaAAAaAAAaAaa
    public void setDesiredPosition(double positionRad) {
//aAAAA
        leftController.setConstraints(normalConstraints);
//aaaaAaaAAaAAaaaAA
        rightController.setConstraints(normalConstraints);
//AaaaaaaAaa
        leftController.setGoal(positionRad);
//AaaAa
        rightController.setGoal(positionRad);
//aAAaAa
    }
//AaaaAAAAAAaaa

//AAaAAA
    public void setDesiredPositionSlow(double positionRad) {
//AaaAaaAaaAAAA
        leftController.setConstraints(climbConstraints);
//AaaaAa
        rightController.setConstraints(climbConstraints);
//aAaAaAAAAAAaA
        leftController.setGoal(positionRad);
//aAaAaaaAAAAAAAaAAaa
        rightController.setGoal(positionRad);
//AAaAAaAaAAAAaA
    }
//AAAAAAaAaAAa

//aaAaaAaa
    public boolean atGoal() {
//aAaAaAaAaaAAAaaA
        return (leftController.atGoal() && rightController.atGoal()) || atGoalOverride;
//AAaaAAAAAaAAAaAa
    }
//AAaaAaaAaAAa

//aAAAA
    public void resetEncoder() {
//AaaAAaA
        leftEncoder.reset();
//AaaAaaAAAaaaAAaAA
        rightEncoder.reset();
//aAAaAaaAaAAAA
    }
//AAAAaAaaaaaAAaa

//AaaaaaAaaAaaaaA
    public void kill() {
//AaAAAAAAaaAAa
        setLeftPercent(0);
//aAAAaaaa
        setRightPercent(0);
//AaAaaaa
        killed = true;
//aAAaAAAAaa
    }
//AaaaAaAAAaAAaaaaaAa

//aAaaaAaAAA
    public boolean getKilled() {
//AAaAAAaaAAA
        return killed;
//aaAaaaAa
    }
//AAaaaAAaAaAaaa

//AAAaAaAAaaAaaAA
    public double getGoal() {
//aAAAaaaaaAAaaAaaAA
        return leftController.getGoal().position;
//aaaAAaaaA
    }
//AaAAAaAaaAAaAaaaAa

//aAAAAAAa
    public void home() {
//aAAAaAaaaaA
        homed = false;
//AaaaaaaA
        homeTimer.reset();
//aaAAaAaaaa
        homeTimer.start();
//AaAAaAAaAAAaaAA
    }
//aaAaAaaaAaAaaaAaaA

//aaaAAaAAaa
    public void setZero() {
//aaaAaAAAAAaAaa
        resetEncoder();
//aAAAA
        setLeftVolts(0);
//AAAAaAAaA
        setRightVolts(0);
//aAaaaAaaAAAaAaaaAA
       
//AAAAaaaAAaaAA
        leftController.reset(leftPositionRad);
//AaAAaAaAaAaAaAaAa
        rightController.reset(rightPositionRad);
//AAAaaaAaaAaAAaAaa

//aaaaAaAaAaaAaA
        leftOverride = false;
//aAaaaaaaAAaAAaAA
        rightOverride = false;
//aaAAaAAAaaAA
    }
//aaaaaAaaAAaa

//AaAaaAAaAAAaaaAAaAa
    public void overrideLeftPercent(double percent) {
//AAaaA
        setLeftPercent(percent);
//AaAaa
        leftOverride = true;
//AAAAaAaaA
    }
//AaaaAaaAaAAaaA

//aAAaaaAaaAA
    public void overrideRightPercent(double percent) {
//AaAaAaaaAaAaAAAAaa
        setRightPercent(percent);
//aAAaaAAaa
        rightOverride = true;
//AAAaAAaaAA
    }
//Aaaaa

//AAAaaaaAaAAaaAa
    public void stop() {
//aaAAaa
        setDesiredPosition(leftPositionRad);
//AaaAAa
    }
//aaaaAaAaaAaAAAAaaA

//aaaaaAaAAaaAAAa
    public void setGoalOverride(boolean override) {
//AaaAAAAAaaAAA
        atGoalOverride = override;
//aAaaaAAaaAaaa
    }
//AaAaAAA

//AAaaAAA
    // Commands
//AaAaAA
    public final Command waitForMove() { return new WaitUntilCommand(this::atGoal); }
//AaAAAAAaAaaaaAaaaa
    public final Command moveToStow () { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.stowPositionRad), this); }
//AaAaAAAAaaAAaAA
    public final Command moveToClimbGrab() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.climbGrabPositionRad), this); }
//aaaaAaAaAaAAAaA
    public final Command moveToIntake() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.intakePositionRad), this); }
//aAaaaaAAAaaAA
    public final Command moveToClimbSwing() { return new InstantCommand(() -> setDesiredPositionSlow(ClimberConstants.climbSwingPositionRad), this); }
//AAaAAaaaAaaA
    public final Command latchRotation() { return new InstantCommand(() -> setDesiredPositionSlow(ClimberConstants.rotationLatchRad), this); }
//AaaAaaaA
    // Maybe add another command for after swinging out to move them slightly back while telescoping down?
//AAAAAaaaAAAaaAAAaaA
    
//aaAaAaAaAAAAAA
}