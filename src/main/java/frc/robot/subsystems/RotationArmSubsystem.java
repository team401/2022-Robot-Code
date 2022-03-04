package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SuperstructureConstants;

public class RotationArmSubsystem extends SubsystemBase {

    public enum Mode {
        Climbing,
        Intaking
    }

    private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(CANDevices.leftRotationMotorID);
    private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(CANDevices.rightRotationMotorID);

    private final DutyCycleEncoder leftEncoder = new DutyCycleEncoder(SuperstructureConstants.leftArmEncoder);
    private final DutyCycleEncoder rightEncoder = new DutyCycleEncoder(SuperstructureConstants.rightArmEncoder);

    private final Encoder quadEncoder = new Encoder(4, 5);

    //PID Controller Constraints 
    private double leftMaxVel = 10.0;
    private double leftMaxAccel = 15.0;
    private double rightMaxVel = 10.0;
    private double rightMaxAccel = 15.0;

    //PID Controllers
    private final ProfiledPIDController leftController = new ProfiledPIDController(3.5, 0, 0.1, 
            new TrapezoidProfile.Constraints(leftMaxVel, leftMaxAccel));

    private final ProfiledPIDController rightController = new ProfiledPIDController(3.5, 0, 0.1, 
            new TrapezoidProfile.Constraints(rightMaxVel, rightMaxAccel));

    //controllers is internal, look at 2021 code ;0;0 
    
    // Goal Values
    private double goalLeftPosition = 0;
    private double goalRightPosition = 0;

    private double tolerance = Units.degreesToRadians(1.5);

    // Boundaries
    // TODO: Change values
    private double frontBoundary = Units.degreesToRadians(55.0);
    private double backBoundary = -Units.degreesToRadians(15.0);

    public RotationArmSubsystem() {

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        quadEncoder.setDistancePerPulse(2* Math.PI / 4096.0);

        leftEncoder.setDistancePerRotation(2 * Math.PI);
        rightEncoder.setDistancePerRotation(2 * Math.PI);

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Left Rotation", getLeftEncoderValue());
        SmartDashboard.putNumber("Right Rotation", getRightEncoderValue());

    }

    // Get encoder value methods
    public double getLeftEncoderValue() {
         if(leftEncoder.getDistance() + ClimberConstants.leftRotationOffset > Math.PI)
            return leftEncoder.getDistance() + ClimberConstants.leftRotationOffset - 2 * Math.PI;
         return (leftEncoder.getDistance() + ClimberConstants.leftRotationOffset);
    }

    public double getRightEncoderValue() {
        if(rightEncoder.getDistance() + ClimberConstants.rightRotationOffset > Math.PI)
            return rightEncoder.getDistance() + ClimberConstants.rightRotationOffset - 2 * Math.PI;
        return rightEncoder.getDistance() + ClimberConstants.rightRotationOffset;
    }

    public void setPIDConstraints(Constraints constraints){
        leftController.setConstraints(constraints);
        rightController.setConstraints(constraints);
    }

    // Set percent methods
    public void setLeftPercent(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setRightPercent(double percent) {
        rightMotor.set(ControlMode.PercentOutput, percent);
    }

    // Set desired position methods
    public void setLeftDesiredPosition(double desiredRadians) {

        goalLeftPosition = desiredRadians;
        double output = leftController.calculate(getLeftEncoderValue(), desiredRadians);
        leftMotor.set(ControlMode.PercentOutput, output);
        SmartDashboard.putNumber("Left Rotation Output", output);

    }

    public void setRightDesiredPosition(double desiredRadians) {

        goalRightPosition = desiredRadians;
        double output = rightController.calculate(getRightEncoderValue(), desiredRadians);
        rightMotor.set(ControlMode.PercentOutput, output);
        SmartDashboard.putNumber("Right Rotation Output", output);

    }

    // Idk what to call methods like atGoal and withinBoundaries
    public boolean atGoal() {
        return Math.abs(getLeftEncoderValue() - goalLeftPosition) <= tolerance && 
                Math.abs(getRightEncoderValue() - goalRightPosition) <= tolerance;
    }

    public boolean withinBoundaries() {
        return (getLeftEncoderValue() >= backBoundary && 
                getRightEncoderValue() >= backBoundary && 
                getLeftEncoderValue() <= frontBoundary && 
                getRightEncoderValue() <= frontBoundary);
    }
    public void resetControllers() {
        leftController.reset(getLeftEncoderValue());
        rightController.reset(getRightEncoderValue());
    }

}