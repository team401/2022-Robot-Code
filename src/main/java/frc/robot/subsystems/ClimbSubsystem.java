package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
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

    private final Encoder rotationQuadEncoder = new Encoder(4, 5);

    //PID Controller Constraints 
    private double leftRotationMaxVel = 10.0;
    private double leftRotationMaxAccel = 15.0;
    private double rightRotationMaxVel = 10.0;
    private double rightRotationMaxAccel = 15.0;

    private double leftTelescopeMaxVel = 1.0;
    private double leftTelescopeMaxAccel = 1.0;
    private double rightTelescopeMaxVel = 1.0;
    private double rightTelescopeMaxAccel = 1.0;


    //PID Controllers
    private final ProfiledPIDController leftRotationController = new ProfiledPIDController(4.0, 0, 0.1, 
            new TrapezoidProfile.Constraints(leftRotationMaxVel, leftRotationMaxAccel));

    private final ProfiledPIDController rightRotationController = new ProfiledPIDController(4.0, 0, 0.1, 
            new TrapezoidProfile.Constraints(rightRotationMaxVel, rightRotationMaxAccel));

    private final ProfiledPIDController leftTelescopeController = new ProfiledPIDController(0.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(leftTelescopeMaxVel, leftTelescopeMaxAccel));
    private final ProfiledPIDController rightTelescopeController = new ProfiledPIDController(0.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(rightTelescopeMaxVel, rightTelescopeMaxAccel));

    //controllers is internal, look at 2021 code ;0;0 
    
    // Goal Values
    private double goalLeftRotationPosition = 0;
    private double goalRightRotationPosition = 0;

    // Goal Tolerance
    // TODO: Change value
    private double rotationTolerance = Units.degreesToRadians(1);
    private double telescopeTolerance = 1.0 / 8.0; //in inches

    // Boundaries
    // TODO: Change values
    private double frontRotationBoundary = Units.degreesToRadians(55.0);
    private double backRotationBoundary = -Units.degreesToRadians(55.0);

    //telescope goal
    private double leftGoal = 0.0;
    private double rightGoal = 0.0;

    private double theoreticalkF =  1.81437 * Units.inchesToMeters(4.0) / 0.71 * 1 * 72.0/17.0 * 70.0;

    public ClimbSubsystem() {

        leftTelescopeMotor.configFactoryDefault();
        leftRotationMotor.configFactoryDefault();
        rightTelescopeMotor.configFactoryDefault();
        rightRotationMotor.configFactoryDefault();

        leftTelescopeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rightTelescopeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        //leftTelescopeMotor.setSelectedSensorPosition(0, 0, 10);
        //rightTelescopeMotor.setSelectedSensorPosition(0, 0, 10);

        leftTelescopeMotor.setNeutralMode(NeutralMode.Brake);
        leftRotationMotor.setNeutralMode(NeutralMode.Brake);
        rightTelescopeMotor.setNeutralMode(NeutralMode.Brake);
        rightRotationMotor.setNeutralMode(NeutralMode.Brake);

        leftTelescopeMotor.selectProfileSlot(0, 0);
        rightTelescopeMotor.selectProfileSlot(0, 0);

       rotationQuadEncoder.setDistancePerPulse(2* Math.PI / 4096.0);

        leftRotationEncoder.setDistancePerRotation(2 * Math.PI);
        rightRotationEncoder.setDistancePerRotation(2 * Math.PI);

        leftTelescopeMotor.config_kP(0, 0);
        leftTelescopeMotor.config_kI(0, 0);
        leftTelescopeMotor.config_kD(0, 0);
        rightTelescopeMotor.config_kP(0, 0);
        rightTelescopeMotor.config_kI(0, 0);
        rightTelescopeMotor.config_kD(0, 0);

        leftTelescopeMotor.setInverted(true);
        rightTelescopeMotor.setInverted(false);
        leftRotationMotor.setInverted(true);
        rightRotationMotor.setInverted(false);

        SmartDashboard.putNumber("P Value", 0.0);
        SmartDashboard.putNumber("I Value", 0.0);
        SmartDashboard.putNumber("D Value", 0.0);
        SmartDashboard.putNumber("velocity", 0.0);
        SmartDashboard.putNumber("acceleration", 0.0);

    }

    @Override
    public void periodic() {


        //SmartDashboard.putNumber("Left Rotation Encoder", getLeftRotationEncoderValue());
        //SmartDashboard.putNumber("Right Rotation Encoder", getRightRotationEncoderValue());

        //SmartDashboard.putBoolean("at goal rot?", atGoal());

        SmartDashboard.putNumber("Left Telescope Encoder", getLeftTelescopeEncoderValue());
        SmartDashboard.putNumber("Right Telescope Encoder", getRightTelescopeEncoderValue());

        SmartDashboard.putNumber("Left Telescope Velocity", getLeftTelescopeVelocity());
        SmartDashboard.putNumber("Right Telescope Velocity", getRightTelescopeVelocity());

        //SmartDashboard.putBoolean("within boundaries?", withinBoundaries());

        //SmartDashboard.putNumber("speed", rotationQuadEncoder.getRate());

        leftTelescopeController.setP(SmartDashboard.getNumber("P Value", 0.0));
        leftTelescopeController.setI(SmartDashboard.getNumber("I Value", 0.0));
        leftTelescopeController.setD(SmartDashboard.getNumber("D Value", 0.0));
        rightTelescopeController.setP(SmartDashboard.getNumber("P Value", 0.0));
        rightTelescopeController.setI(SmartDashboard.getNumber("I Value", 0.0));
        rightTelescopeController.setD( SmartDashboard.getNumber("D Value", 0.0));

        leftTelescopeController.setConstraints(new TrapezoidProfile.Constraints(
            SmartDashboard.getNumber("velocity", 1.0), SmartDashboard.getNumber("acceleration", 1.0)));
        rightTelescopeController.setConstraints(new TrapezoidProfile.Constraints(
            SmartDashboard.getNumber("velocity", 1.0), SmartDashboard.getNumber("acceleration", 1.0)));
    
    }

    // Get encoder value methods
    public double getLeftRotationEncoderValue() {
         if(leftRotationEncoder.getDistance() + ClimberConstants.leftRotationOffset > Math.PI)
            return leftRotationEncoder.getDistance() + ClimberConstants.leftRotationOffset - 2 * Math.PI;
         return (leftRotationEncoder.getDistance() + ClimberConstants.leftRotationOffset);
        //dreturn leftRotationEncoder.getDistance();
    }

    public double getRightRotationEncoderValue() {
        if(rightRotationEncoder.getDistance() + ClimberConstants.rightRotationOffset > Math.PI)
            return rightRotationEncoder.getDistance() + ClimberConstants.rightRotationOffset - 2 * Math.PI;
        return rightRotationEncoder.getDistance() + ClimberConstants.rightRotationOffset;
        //return rightRotationEncoder.getDistance();
    }

    //in linear with inches
    public double getLeftTelescopeEncoderValue() {
        return leftTelescopeMotor.getSelectedSensorPosition() / 4096.0 * Math.PI * 2 
            * ClimberConstants.linearConversion;
    } 

    //in linear with inches
    public double getRightTelescopeEncoderValue() {
        return rightTelescopeMotor.getSelectedSensorPosition() / 4096.0 * Math.PI * 2
            * ClimberConstants.linearConversion;
    }

    public void changeRotationPIDConstraints(Constraints constraints){
        leftRotationController.setConstraints(constraints);
        rightRotationController.setConstraints(constraints);
    }

    //in linear with inches/s
    public double getLeftTelescopeVelocity() {
        // * 10 converts from units per 100ms to units per 1s
        return leftTelescopeMotor.getSelectedSensorVelocity() * 10 / 4096.0 * Math.PI * 2
            * ClimberConstants.linearConversion;
    }

    //in linear with inches/s
    public double getRightTelescopeVelocity() {
        // * 10 converts from units per 100ms to units per 1s
        return rightTelescopeMotor.getSelectedSensorVelocity() * 10 / 4096.0 *  Math.PI * 2 
            * ClimberConstants.linearConversion;
    }

    // Get amp value methods
    public double getLeftTelescopeAmps() {
        return leftTelescopeMotor.getStatorCurrent();
    } 

    public double getRightTelescopeAmps() {
        return rightTelescopeMotor.getStatorCurrent();
    }

    //resets the telescope encoders
    public void resetLeftTelescopeEncoder() {
        leftTelescopeMotor.setSelectedSensorPosition(0.0);
    }

    public void resetRightTelescopeEncoder() {
        rightTelescopeMotor.setSelectedSensorPosition(0.0);
    }

    // Set percent methods
    public void setLeftRotationPercent(double percent) {
        leftRotationMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setRightRotationPercent(double percent) {
        rightRotationMotor.set(ControlMode.PercentOutput, percent);
    }

    public void runLeftTelescopePercent() {
        setLeftTelescopePercent(0.25);
    }

    public void runRightTelescopePercent() {
        setRightTelescopePercent(0.25);
    }

    public void retractLeftTelescope() {
        setLeftTelescopePercent(-0.25);
    }

    public void retractRightTelescope() {
        setRightTelescopePercent(-0.25);
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
        SmartDashboard.putNumber("desired", desiredRadians);
        double output = leftRotationController.calculate(getLeftRotationEncoderValue(), desiredRadians);
        SmartDashboard.putNumber("output", output);
        leftRotationMotor.set(ControlMode.PercentOutput, output);

    }

    public void setRightDesiredRotationPosition(double desiredRadians) {

        goalRightRotationPosition = desiredRadians;
        double output = rightRotationController.calculate(getRightRotationEncoderValue(), desiredRadians);
        rightRotationMotor.set(ControlMode.PercentOutput, output);

    }

    //value is passed in in inches
    //TODO: check set method
    public void setLeftDesiredTelescopePosition(double desiredPosition){
        leftGoal = desiredPosition;
        double output = leftTelescopeController.calculate(getLeftTelescopeEncoderValue(), leftGoal);

        leftTelescopeMotor.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void setRightDesiredTelescopePosition(double desiredPosition){
        rightGoal = desiredPosition;
        double output = rightTelescopeController.calculate(getRightTelescopeEncoderValue(), rightGoal);

        rightTelescopeMotor.set(TalonSRXControlMode.PercentOutput, output);
    }

    // Idk what to call methods like atGoal and withinBoundaries
    public boolean atGoalRotation() {
        return Math.abs(getLeftRotationEncoderValue() - goalLeftRotationPosition) <= rotationTolerance && 
                Math.abs(getRightRotationEncoderValue() - goalRightRotationPosition) <= rotationTolerance;
    }

    public boolean atGoalTelescope() {
        return Math.abs(getLeftTelescopeEncoderValue() - leftGoal) <= telescopeTolerance &&
                Math.abs(getRightTelescopeEncoderValue() - rightGoal) <= telescopeTolerance;
    }

    public boolean withinBoundariesRotation() {
        return (getLeftRotationEncoderValue() >= backRotationBoundary && 
                getRightRotationEncoderValue() >= backRotationBoundary && 
                getLeftRotationEncoderValue() <= frontRotationBoundary && 
                getRightRotationEncoderValue() <= frontRotationBoundary);
    }

    public boolean withinBoundariesTelescope() {
        return (getLeftTelescopeEncoderValue() >= 0.0 &&
                getRightTelescopeEncoderValue() >= 0.0 &&
                getLeftTelescopeEncoderValue() <= ClimberConstants.maxHeight &&
                getRightTelescopeEncoderValue() <= ClimberConstants.maxHeight);
    }

    public void resetRotationControllers() {
        leftRotationController.reset(getLeftRotationEncoderValue());
        rightRotationController.reset(getRightRotationEncoderValue());
    }

    public void resetTelescopeControllers() {
        leftTelescopeController.reset(getLeftTelescopeEncoderValue());
        rightTelescopeController.reset(getRightTelescopeEncoderValue());
    }

}