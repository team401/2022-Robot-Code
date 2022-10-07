//AAaaaAaAAAAAAaAA
package frc.robot.subsystems;
//aaAaAAAAA

//aaaAa
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//AaAaAaAAAAAaaaaAAAa

//AAAaaaa
import com.revrobotics.CANSparkMax;
//aAAAaA
import com.revrobotics.RelativeEncoder;
//aaaaAaAA
import com.revrobotics.CANSparkMax.IdleMode;
//aAAAAaaAaAaAAaaaaA
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//aAaaaAAAAaAAaAAAA

//AaAaaaaaAa
import frc.robot.Constants;
//aaaaAAaAaaaaAaA
import frc.robot.Constants.CANDevices;
//AaaaaaaAaaaA
import frc.robot.Constants.ClimberConstants;
//aaaaaAaaaa

//aaAAAaaAaAaAaaAa
import edu.wpi.first.wpilibj.DriverStation;
//aAAAaaAAaAAA
import edu.wpi.first.wpilibj.Timer;
//AAaAAAAAAaAaAaaaaA
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//AAaAaaAAAAaaa
import edu.wpi.first.wpilibj2.command.Command;
//aAAAAAAaaAaAa
import edu.wpi.first.wpilibj2.command.InstantCommand;
//aaaAAAaaAaaAaAa
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//aAAAAaaaaaa

//aAaAaAAAaaAaaAAAA
import edu.wpi.first.math.controller.ProfiledPIDController;
//aAaaAaAAAAaaaAaaA
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//AaaAaaaAAaAAAaaAAaa

//AaAAAA
import com.playingwithfusion.TimeOfFlight;
//aaaAAaaAaAAa

//AAaAAaaaaaA
public class Telescopes extends SubsystemBase {
//AAaAaaAAAaaAa

//AaAaAAaAAAAaaaAAaaA
    private final CANSparkMax leftMotor;
//AaAAAa
    private final CANSparkMax rightMotor;
//aaaaaAAAaAaAA

//aAAaaaAaA
    private final RelativeEncoder leftEncoder;
//AAAAAAaAAAAAaaAA
    private final RelativeEncoder rightEncoder;
//aAAAA

//aaAaAaaaa
    private final TimeOfFlight leftLidar;
//AaaAAaAaaaaaA
    private final TimeOfFlight rightLidar;
//AAaaAaaAAaaAAAa

//aAAAa
    public double leftLidarPositionM;
//aaaAA
    public double rightLidarPositionM;
//aAAaaAAa
    public double leftEncoderPositionM;
//aaaaaaaAaa
    public double rightEncoderPositionM;
//AAaAAaaa
    public double leftVelocityMPerS;
//aAAaAAAaaaaA
    public double rightVelocityMPerS;
//aAAAAaaAAaAAaAAaA
    public double leftCurrent;
//aAaaAAAAaaaaaa
    public double rightCurrent;
//AaaaaaaAAaaaaAaAAA

//aAaaaaAAAAaAAa
    private final ProfiledPIDController leftController = new ProfiledPIDController(
//AAaAaAAAaAAa
        ClimberConstants.telescopeArmKp, 0, ClimberConstants.telescopeArmKd,
//aAaAAaaAaAa
        new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocityM, ClimberConstants.telescopeAccelerationM)
//aAaaaaAAAAaaAaaaAaa
    );
//AaAAAaAA

//aAAaaaAa
    private final ProfiledPIDController rightController = new ProfiledPIDController(
//AaaAaA
        ClimberConstants.telescopeArmKp, 0, ClimberConstants.telescopeArmKd,
//aaaAaAAa
        new TrapezoidProfile.Constraints(ClimberConstants.telescopeCruiseVelocityM, ClimberConstants.telescopeAccelerationM)
//aaAAaAaAA
    );
//AaaaaAaaAaaaaAaAAA

//AaAAaaaaaaaAa
    private final double deltaTime = 0.02; // 20ms between each cycle
//AAAAa

//aaAaaAaAAAAAAaAaA
    private double desiredPositionM = 0; // position the telescopes attempt to reach in periodic
//aaaAAAAaAAA

//AAAaaaaaAaAAAaaAa
    private boolean motorOverride = false; // true = periodic doesn't set motor voltage/percent
//aaaaAaaaAAAaAAa
    private boolean atGoalOverride = false; // true = atGoal() always returns true
//AaaAAaAa

//aaAaaaAaAA
    private boolean leftHomed = false;
//AAaAaaaAAaAaaaaAAaa
    private boolean rightHomed = false;
//AAAaAAAaaAaaaAaa

//aaAAaA
    private final Timer leftHomeTimer = new Timer();
//aAAaAAaA
    private final Timer rightHomeTimer = new Timer();
//AaAaAaAAAaAAaAaAAAa

//aaAaAaaA
    public Telescopes() {
//AAAaaaAaAaaAa

//aaaAA
        // Initialization
//aaaAAa
        leftMotor = new CANSparkMax(CANDevices.leftTelescopingMotorID, MotorType.kBrushless);
//AAaAAaAaa
        rightMotor = new CANSparkMax(CANDevices.rightTelescopingMotorID, MotorType.kBrushless);
//AAaaAAaaaAAAaAA

//aaaaa
        leftEncoder = leftMotor.getEncoder();
//AaaAAaaaAAAaAAAaaa
        rightEncoder = rightMotor.getEncoder();
//aaaAAAaAAaAaaAAaA

//aAaAAaaaaaaA
        leftLidar = new TimeOfFlight(CANDevices.leftLidar);
//AaaAAAaaaAaaAAAaa
        rightLidar = new TimeOfFlight(CANDevices.rightLidar);
//aAaaAAaaAAaaa

//AaAaaaAAaAaAaaAa
        // Options
//AAaaAAAAAaaaAaA
        leftMotor.setSmartCurrentLimit(80);
//aAaaAAaA
        rightMotor.setSmartCurrentLimit(80);
//aAaaAaAAAaaAAAAA

//aAaaaaaaaAAaAAAA
        leftMotor.setIdleMode(IdleMode.kBrake);
//aAAaaaAAAaaAaAaAA
        rightMotor.setIdleMode(IdleMode.kBrake);
//AaAaAaa
        
//AAAAaaaaAaAaa
        leftEncoder.setPositionConversionFactor(ClimberConstants.telescopeConversionFactor);
//AaaAAaaAaaaaaaA
        rightEncoder.setPositionConversionFactor(ClimberConstants.telescopeConversionFactor);
//aaaAaaAaAaAAAa

//aAaaAAAAAA
        leftLidar.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
//aaAAaAAAa
        rightLidar.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
//aAaaAAA

//AaAaa
        leftController.setGoal(ClimberConstants.telescopeHomePositionM);
//AaaAA
        rightController.setGoal(ClimberConstants.telescopeHomePositionM);
//AAaAaaaAAAA

//AAAaA
        leftController.setTolerance(ClimberConstants.telescopeGoalToleranceM);
//AAAaAaaAAaAaAAaa
        rightController.setTolerance(ClimberConstants.telescopeGoalToleranceM);
//AaaaaaAaA

//aAAaaaaAAaAAAA
        leftHomeTimer.start();
//AaAAaAaa
        rightHomeTimer.start();
//aAAAaAaaAAaAaaAAa

//aAAaaaa
    }
//AAAaaaAAAa

//aaaAAAaaAAaAAAAAAAa
    @Override
//AaAaAaAaaaAaAa
    public void periodic() {
//aaaaAaAaAaA

//aAAaaaAAAAAA
        if (leftLidar.getStatus() == TimeOfFlight.Status.Valid)
//AAaaA
            leftLidarPositionM = leftLidar.getRange() - ClimberConstants.telescopeOffsetM;
//aaAaaAaaAAaaAAAAAAa
        if (rightLidar.getStatus() == TimeOfFlight.Status.Valid)
//AAaaaAaaaAaaaAAaAA
            rightLidarPositionM = rightLidar.getRange() - ClimberConstants.telescopeOffsetM;
//AaaAAAAaaaAAaAaAaA

//AaAaAAA
        leftEncoderPositionM = leftEncoder.getPosition();
//AAaAAAaAaAaAaAa
        rightEncoderPositionM = rightEncoder.getPosition();
//aaaaaAAAAAA

//aAaAAaaaAaAaaaAaa
        leftVelocityMPerS = leftEncoder.getVelocity() / 60.0;
//AAaaaaAAa
        rightVelocityMPerS = rightEncoder.getVelocity() / 60.0;
//aAAaaaaAaAaAAAa

//aaAaA
        leftCurrent = leftMotor.getOutputCurrent();
//aaAaaa
        rightCurrent = rightMotor.getOutputCurrent();
//aAaaAaaaa

//aAaAAAaAA
        long m_Start = System.currentTimeMillis();
//AAAAAAAAAaAAAaAa

//aAaaAAaAaaAaaaA
        if (!DriverStation.isEnabled()) {
//aAAaAAAaaaaaAAAAaa
            if (isLeftLidarValid())
//aAaAaaaaaaAAaAaaAA
                leftController.reset(leftLidarPositionM);
//aaaAaaAaaaAAAaaaAA
            else
//AaAaAaaAAA
                leftController.reset(leftEncoderPositionM);
//aAaaaaaaAaAaAAaaaa
            if (isRightLidarValid())
//AaAAA
                rightController.reset(rightLidarPositionM);
//aAaAAAAaaaaaAa
            else
//AaaAaaAaaAaaaA
                rightController.reset(rightEncoderPositionM);
//AAaAAaAaAAaa
            
//AaAaAaaaAa
            leftHomeTimer.reset();
//AAaAAAAaaAAAaAAAaA
            rightHomeTimer.reset();
//AaaaaaaAaaaaaaAAAA
        }
//aAAAAAAa

//AAaaAAAaAAaaaAaaaa
        if (DriverStation.isEnabled() && !motorOverride) {
//AaAAAAAaAaAA
            // Left
//aAAaaAAaaAAAAAAa
            if (isLeftLidarValid()) {
//aaaaAAAAaAAaA
                double output = leftController.calculate(leftLidarPositionM, desiredPositionM);
//AaaaAaaaAaAAaAAaAA
                output -= output < 0 ? 0.7 : 0; // FF
//aaAAAaaAAAa
                setLeftVolts(output);
//aaaAaaaaaAAaA

//aAAaa
                leftHomed = true;
//aaaaa
                if (Math.abs(leftLidarPositionM - leftEncoderPositionM) > ClimberConstants.telescopeGoalToleranceM)
//aAAAAaaaAaAaAaaaA
                    setLeftEncoder(leftLidarPositionM);
//AaAAaAaaAAAaAaAA
            }
//AAAaAaAaAAAaaA
            else {
//aAAaaAAaaAAAaaAaAAA
                if (leftHomed) {
//aAaAaAaAaaAaaaaaAaA
                    double output = leftController.calculate(leftEncoderPositionM, desiredPositionM);
//AaAaaaAaAA
                    output -= output < 0 ? 0.7 : 0; // FF
//aAAaa
                    setLeftVolts(output);
//aAaAAaaaAAAAaaAA
                }
//aAaAaaAaaAaA
                else {
//AAaAAAAAaAaaaAA
                    if (Math.abs(leftVelocityMPerS) > ClimberConstants.telescopeHomingThresholdMPerS) {
//aaAaAaA
                        leftHomeTimer.reset();
//aAaAaA
                    }
//AAAaaAAAaAAaAaaa
                    else if (leftHomeTimer.hasElapsed(ClimberConstants.homingTimeS)){
//aAAaAAaAAaaaAaAaAAa
                        leftHomed = true;
//AaAaAaAAaaAA
                        setLeftEncoder(0);
//AAaAA
                        setLeftVolts(0);
//aaAaaa
                        leftController.reset(leftEncoderPositionM);
//AaaAaaaa
                    }
//AaAaaAAaAaAAaA
                    else {
//aaAAAAA
                        setLeftVolts(ClimberConstants.telescopeHomingVolts);
//aaaaAaA
                    }
//AAAaAaAaaaaaAAa
                }
//AaaaAaaaaAAAaAaAa
            }
//aAAaaAAaAaaaaAAa

//aaAaaAAA
            // Right
//aaaAAaaaAaaAaA
            if (isRightLidarValid()) {
//AaAaAaaA
                double output = rightController.calculate(rightLidarPositionM, desiredPositionM);
//aaaAAAaaAAA
                output -= output < 0 ? 0.7 : 0; // FF
//aAAaaaaa
                setRightVolts(output);
//aAaaaaAAaaaAAAaaaAa

//AaAaaaaaAAaA
                rightHomed = true;
//AAAaAAAAaAA
                if (Math.abs(rightLidarPositionM - rightEncoderPositionM) > ClimberConstants.telescopeGoalToleranceM)
//aaAAaAaAaaaAAa
                    setRightEncoder(rightLidarPositionM);
//aAAAAaAaaAAAAa
            }
//AaAaAaaAaAAaaaa
            else {
//AAaAA
                if (rightHomed) {
//aAaAAAAaAAaAa
                    double output = rightController.calculate(rightEncoderPositionM, desiredPositionM);
//aaaAaAAaAA
                    output -= output < 0 ? 0.7 : 0; // FF
//aaaAaaAAaaA
                    SmartDashboard.putNumber("Right Output", output);
//aAAaaAAAaAAa
                    setRightVolts(output);
//aAAaA
                }
//AAaaaaAaAAAaa
                else {
//aaAAAAaAAa
                    if (Math.abs(rightVelocityMPerS) > ClimberConstants.telescopeHomingThresholdMPerS) {
//AAAaaaaA
                        rightHomeTimer.reset();
//AAaAAAaAaaAa
                    }
//Aaaaa
                    else if (rightHomeTimer.hasElapsed(ClimberConstants.homingTimeS)){
//AaAaA
                        rightHomed = true;
//AaaaAAAAaaAaAA
                        setRightEncoder(0);
//AAAAAAa
                        setRightVolts(0);
//AaAaaAaAaaaAaaaAaa
                        rightController.reset(rightEncoderPositionM);
//AAaAaAAaAaaAaAAaa
                    }
//aaaAaAAa
                    else {
//aaaAAaAaAaaAaAA
                        setRightVolts(ClimberConstants.telescopeHomingVolts);
//AaaAAAAAAAaaaaaA
                    }
//aaAAaAaAaaAaaaa
                }
//AAAaa
            }
//AAAaAAaAAaAaaAAaAAa
        }
//aAAAaaa

//AaAAaAaa
        SmartDashboard.putBoolean("Telescopes At Goal", atGoal());
//aAAAAAaAaAAaa
        SmartDashboard.putNumber("Left Homed", leftHomed ? 1 : 0);
//aaaaAAA
        SmartDashboard.putNumber("Right Homed", rightHomed ? 1 : 0);
//AAAAAAaaAAaAAa
        SmartDashboard.putNumber("Right Pos M", rightEncoderPositionM);
//aAAaaaAaaA
        SmartDashboard.putNumber("Left Pos M", leftEncoderPositionM);
//aAAaaaAaaaAA
        SmartDashboard.putNumber("Desired Right Pos M", desiredPositionM);
//AAaaAaaAa

//aAAaaAaaaaaaAa
    }
//AAaAAA

//AAAaaAaAAA
    public void setLeftEncoder(double positionM) {
//AAaaaa
        leftEncoder.setPosition(positionM);
//aAaAAAAaAaaAaAA
    }
//AaAAaaAA

//aaaaaAaaaAaAAaA
    public void setRightEncoder(double positionM) {
//aaaAAAaAA
        rightEncoder.setPosition(positionM);
//AAAaAAAaAaaAaAAAAaA
    }
//AaAAAaaAAaAAaaAA

//AAAAAAAAaA
    public void setLeftVolts(double volts) {
//AAaaAAAaaAAAAaaa
        leftMotor.setVoltage(volts);        
//aAaAaaaAa
    }
//aaAaaAAa

//AaaAaAaAAaAAaaa
    public void setRightVolts(double volts) {
//AAaaAaaAA
        rightMotor.setVoltage(volts);        
//AaaaaAAAAAaAaaA
    }
//aaAaAAAAaaAaaAAa

//aaaaA
    public double getLeftCurrentDraw() {
//aaaaAa
        return leftMotor.getOutputCurrent();
//aAaAAaAAAAa
    }
//AAAaaaaAaaAA
    
//aaAaaAAAaaaAaAaaAa
    public double getRightCurrentDraw() {
//AaAaAAaaAaAAAAAaaA
        return rightMotor.getOutputCurrent();
//aAaaaaAAAaAaAAAA
    }
//AaAAAa

//AaaaAaAAA
    public boolean isLeftLidarValid() {
//aAAAA
        return leftLidar.getStatus() == TimeOfFlight.Status.Valid;
//aaaaaaA
    }
//aaaAAAaaaAaA

//AaaAAaaaAa
    public boolean isRightLidarValid() {
//aAaaAAaaaAa
        return rightLidar.getStatus() == TimeOfFlight.Status.Valid;
//aaaaAAAaAAa
    }
//aaAaAaaaa

//aAAAa
    public void setLeftPercent(double percent) {
//aaAaaa
        setLeftVolts(percent * 12);
//aaAAaAAaAaaaaAa
    }
//AaAaaaAaAAAaaAAA

//AAaAaaaaAAaAAAaAAAa
    public void setRightPercent(double percent) {
//AAaAaaaAaAAaaaaAaAa
        setRightVolts(percent * 12);
//AAAaaAaaaaaA
    }
//Aaaaa

//aAAAAAaAaaaAAaAAaa
    public void setDesiredPosition(double positionM) {
//aAAaaaaaAAaa
        desiredPositionM = positionM;
//AAaaAa
    }
//aaAaAAAAAAaaa

//AAaaaA
    public void jogUp() {
//aAAAaAaaA
        desiredPositionM += deltaTime * ClimberConstants.telescopeCruiseVelocityM;
//AAAaAaAAaaaaaAAaaA
        if (desiredPositionM > ClimberConstants.telescopeMaxPositionM)
//AAaAAa
            desiredPositionM = ClimberConstants.telescopeMaxPositionM;
//aAaAaaaAAaAaAaaaaaA
    }
//AAaAaAaAaA

//aAAaaAAAAaAAAaAAAa
    public void jogDown() {
//aAaAaAaAAAaaaaaaAa
        desiredPositionM -= deltaTime * ClimberConstants.telescopeCruiseVelocityM;
//AAaaAaa
        if (desiredPositionM < ClimberConstants.telescopeHomePositionM)
//AaAaAAAAAAa
            desiredPositionM = ClimberConstants.telescopeHomePositionM;
//aAAAaAaAAAaA
    }
//aAaaaaaaaAaaa

//aaAaAaAaaaaAaAaa
    public boolean atGoal() {
//AAaAaAA
        return atGoalOverride || (leftController.atGoal() && rightController.atGoal());
//aAaaAaaAaAaaaAa
    }
//aaaAAaAAaAaaaAAa

//AaaaaAA
    public boolean passedRotationSafePosition() {
//AaAaaAAAA
        double leftPositionM = isLeftLidarValid() ? leftLidarPositionM : leftEncoderPositionM;
//aAaAAaaAaaAAAaaA
        double rightPositionM = isRightLidarValid() ? rightLidarPositionM : rightEncoderPositionM;
//aaaaAaaAA
        return leftPositionM <= ClimberConstants.telescopeRotationSafePositionM &&
//AaAAAaAaa
                rightPositionM <= ClimberConstants.telescopeRotationSafePositionM;
//AAAAaaaaaA
    }
//AaAaaAAaA

//AaaAaa
    public void stop() {
//AaaaAa
        setDesiredPosition(isLeftLidarValid() ? leftLidarPositionM : leftEncoderPositionM);
//AAAaaa
    }
//AaAAaAAAA

//AAAaaAaaaAaaaaAAa
    public void setMotorOverride(boolean override) {
//aaaaaaaaAAAaaaAa
        motorOverride = override;
//AAAAAaaAAAAaAaaAA
        setLeftVolts(0);
//aaAAAAAAAAa
        setRightVolts(0);
//aaaAAAaAaAAAaaaaa
    }
//AaaaAaAaAa

//aAAaAaAAaaaAaa
    public void setAtGoalOverride(boolean override) {
//aaaaaaaAAAaAa
        atGoalOverride = override;
//aAaaAaaA
    }
//aaaaaaAAaA

//aAAaAAa
    // Commands
//aaAaaaaAaAaa
    public final Command waitForMove() { return new WaitUntilCommand(this::atGoal); }
//aAaAAaaaaAaaaaa
    public final Command waitForRotationSafePosition() { return new WaitUntilCommand(this::passedRotationSafePosition); }
//AAAAAAAaAaAaAAaA
    public final Command moveToPop() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePopAboveRungM), this); }
//aaaAAaAaaAaaaa
    public final Command moveToFull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeMaxPositionM), this); }
//AaaAAAaaaaAAAa
    public final Command moveToLatch() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeLatchM), this); }
//aaaAaAAAaaaaAa
    public final Command moveToPull() { return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopePullPositionM), this); }
//AAAAAaAaaaaAAA
    public final Command moveToSwing() {return new InstantCommand(() -> setDesiredPosition(ClimberConstants.telescopeSwingPositionM), this); }
//aAAaAaAaAaAaAAaaaA
    
//aAAAAaaaaAaAaaaA
}
