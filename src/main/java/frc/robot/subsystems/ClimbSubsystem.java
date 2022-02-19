package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SuperstructureConstants;

public class ClimbSubsystem extends SubsystemBase {

    //sets up our two climbing arms (each which has two motors: one who can rotate and one who can't)
    private final WPI_TalonSRX leftTelescopeMotor = new WPI_TalonSRX(CANDevices.leftTelescopingMotorID);
    private final WPI_TalonSRX rightTelescopeMotor = new WPI_TalonSRX(CANDevices.rightTelescopingMotorID);
    private final WPI_TalonSRX leftRotationMotor = new WPI_TalonSRX(CANDevices.leftRotationMotorID);
    private final WPI_TalonSRX rightRotationMotor = new WPI_TalonSRX(CANDevices.rightRotationMotorID);

    private final DutyCycleEncoder leftRotationEncoder = new DutyCycleEncoder(SuperstructureConstants.leftArmEncoder);
    private final DutyCycleEncoder rightRotationEncoder = new DutyCycleEncoder(SuperstructureConstants.rightArmEncoder);

    private final Encoder quadratureEncoder = new Encoder(4,5);

    //PID Controller Constraints 
    private double leftRotationMaxVel = Math.PI * 2;
    private double leftRotationMaxAccel = 5;
    private double rightRotationMaxVel = Math.PI * 2;
    private double rightRotationMaxAccel = 5;

    //PID Controllers
    private final ProfiledPIDController leftRotationController = new ProfiledPIDController(0.35, 0, 0, 
            new TrapezoidProfile.Constraints(leftRotationMaxVel, leftRotationMaxAccel));

    private final ProfiledPIDController rightRotationController = new ProfiledPIDController(0.35, 0, 0, 
            new TrapezoidProfile.Constraints(rightRotationMaxVel, rightRotationMaxAccel));


    
    // Goal Values
    private double goalLeftRotationPosition = 0;
    private double goalRightRotationPosition = 0;

    // Goal Tolerance
    // TODO: Change value
    private double tolerance = Units.degreesToRadians(1);

    // Boundaries
    // TODO: Change values
    private double frontRotationBoundary = 0;
    private double backRotationBoundary = 0;

    public ClimbSubsystem() {

        leftTelescopeMotor.configFactoryDefault();
        leftRotationMotor.configFactoryDefault();
        rightTelescopeMotor.configFactoryDefault();
        rightRotationMotor.configFactoryDefault();

        leftTelescopeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        rightTelescopeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

        leftTelescopeMotor.setSelectedSensorPosition(0, 0, 10);
        rightTelescopeMotor.setSelectedSensorPosition(0, 0, 10);

        leftTelescopeMotor.setNeutralMode(NeutralMode.Brake);
        leftRotationMotor.setNeutralMode(NeutralMode.Brake);
        rightTelescopeMotor.setNeutralMode(NeutralMode.Brake);
        rightRotationMotor.setNeutralMode(NeutralMode.Brake);

        quadratureEncoder.setDistancePerPulse(2* Math.PI / 4096.0);

        //leftRotationMotor.setNeutralMode(NeutralMode.Brake);

        // TODO: Do we need to change vsalues?
        leftRotationEncoder.setDistancePerRotation(2 * Math.PI);
        rightRotationEncoder.setDistancePerRotation(2 * Math.PI);

        leftTelescopeMotor.setInverted(true);
        leftRotationMotor.setInverted(true);

        SmartDashboard.putNumber("P Value", 0.0);
        SmartDashboard.putNumber("I Value", 0.0);
        SmartDashboard.putNumber("D Value", 0.0);
        SmartDashboard.putNumber("velocity", 0.0);
        SmartDashboard.putNumber("acceleration", 0.0);

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("quadrature encoder vel", quadratureEncoder.getRate());

        SmartDashboard.putNumber("Left Rotation Encoder", getLeftRotationEncoderValue());
        SmartDashboard.putNumber("Right Rotation Encoder", getRightRotationEncoderValue());

        SmartDashboard.putBoolean("at goal rot?", atGoal());

        /*SmartDashboard.putNumber("Left Telescope Encoder", getLeftTelescopeEncoderValue());
        SmartDashboard.putNumber("Right Telescope Encoder", getRightTelescopeEncoderValue());

        SmartDashboard.putNumber("Left Telescope Velocity", getLeftTelescopeVelocity());
        SmartDashboard.putNumber("Right Telescope Velocity", getRightTelescopeVelocity());*/

        leftRotationController.setP(SmartDashboard.getNumber("P Value", 0.0));
        leftRotationController.setI(SmartDashboard.getNumber("I Value", 0.0));
        leftRotationController.setD(SmartDashboard.getNumber("D Value", 0.0));
        rightRotationController.setP(SmartDashboard.getNumber("P Value", 0.0));
        rightRotationController.setI(SmartDashboard.getNumber("I Value", 0.0));
        rightRotationController.setD(SmartDashboard.getNumber("D Value", 0.0));

        leftRotationController.setConstraints(new 
            TrapezoidProfile.Constraints(SmartDashboard.getNumber("velocity", 1.0), 
            SmartDashboard.getNumber("acceleration", 1.0)));

        rightRotationController.setConstraints(new 
            TrapezoidProfile.Constraints(SmartDashboard.getNumber("velocity", 1.0), 
            SmartDashboard.getNumber("acceleration", 1.0)));

    }

    // Get encoder value methods
    public double getLeftRotationEncoderValue() {
       // if (leftRotationEncoder.getDistance() + ClimberConstants.leftRotationOffset > Math.PI) 
         //   return ((leftRotationEncoder.getDistance() + ClimberConstants.leftRotationOffset) % 2* Math.PI)- 2 * Math.PI;
         if(leftRotationEncoder.getDistance() + ClimberConstants.leftRotationOffset > Math.PI)
            return leftRotationEncoder.getDistance() + ClimberConstants.leftRotationOffset - 2 * Math.PI;
         return (leftRotationEncoder.getDistance() + ClimberConstants.leftRotationOffset);
        //dreturn leftRotationEncoder.getDistance();
    }

    public double getRightRotationEncoderValue() {
        //if (rightRotationEncoder.getDistance() + ClimberConstants.rightRotationOffset > Math.PI) 
          //  return ((rightRotationEncoder.getDistance() + ClimberConstants.rightRotationOffset) % 2* Math.PI)- 2 * Math.PI;
        if(rightRotationEncoder.getDistance() + ClimberConstants.rightRotationOffset > Math.PI)
            return rightRotationEncoder.getDistance() + ClimberConstants.rightRotationOffset - 2 * Math.PI;
        return rightRotationEncoder.getDistance() + ClimberConstants.rightRotationOffset;
        //return rightRotationEncoder.getDistance();
    }

    public double getLeftTelescopeEncoderValue() {
        return leftTelescopeMotor.getSelectedSensorPosition();
    } 

    public double getRightTelescopeEncoderValue() {
        return rightTelescopeMotor.getSelectedSensorPosition();
    }

    public double getLeftTelescopeVelocity() {
        // * 10 converts from units per 100ms to units per 1s
        return leftTelescopeMotor.getSelectedSensorVelocity() * 10;
    }

    public double getRightTelescopeVelocity() {
        // * 10 converts from units per 100ms to units per 1s
        return rightTelescopeMotor.getSelectedSensorVelocity() * 10;
    }

    // Get amp value methods
    public double getLeftTelescopeAmps() {
        return leftTelescopeMotor.getStatorCurrent();
    } 

    public double getRightTelescopeAmps() {
        return rightTelescopeMotor.getStatorCurrent();
    }

    // Set percent methods
    public void setLeftRotationPercent(double percent) {
        SmartDashboard.putNumber("Left Percent Output", percent);
        leftRotationMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setRightRotationPercent(double percent) {
        SmartDashboard.putNumber("right Percent Output", percent);
        rightRotationMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setLeftTelescopePercent(double percent) {
        leftTelescopeMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setRightTelescopePercent(double percent) {
        rightTelescopeMotor.set(ControlMode.PercentOutput, percent);
    }

    // Set desired position methods
    public void setLeftDesiredRotationPosition(double desiredRadians) {

        goalLeftRotationPosition = desiredRadians;
        double output = leftRotationController.calculate(getLeftRotationEncoderValue(), desiredRadians);
        leftRotationMotor.set(ControlMode.PercentOutput, output);

    }

    public void setRightDesiredRotationPosition(double desiredRadians) {

        goalRightRotationPosition = desiredRadians;
        double output = rightRotationController.calculate(getRightRotationEncoderValue(), desiredRadians);
        rightRotationMotor.set(ControlMode.PercentOutput, output);

    }

    // Idk what to call methods like atGoal and withinBoundaries
    public boolean atGoal() {
        return Math.abs(getLeftRotationEncoderValue() - goalLeftRotationPosition) <= tolerance && 
                Math.abs(getRightRotationEncoderValue() - goalRightRotationPosition) <= tolerance;
    }

    public boolean withinBoundaries() {
        return getLeftRotationEncoderValue() > backRotationBoundary && 
                getRightRotationEncoderValue() > backRotationBoundary && 
                getLeftRotationEncoderValue() < frontRotationBoundary && 
                getLeftRotationEncoderValue() < frontRotationBoundary;
    }

    public void resetControllers() {
        leftRotationController.reset(getLeftRotationEncoderValue());
        rightRotationController.reset(getRightRotationEncoderValue());
    }

}