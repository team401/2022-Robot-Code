package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

public class TelescopeArmSubsystem extends SubsystemBase {

    //sets up our two climbing arms
    private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(CANDevices.leftTelescopingMotorID);
    private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(CANDevices.rightTelescopingMotorID);

    //PID Controller Constraints
    private double leftMaxVel = 5.0;
    private double leftMaxAccel = 5.0;
    private double rightMaxVel = 5.0;
    private double rightMaxAccel = 5.0;

    //PID Controllers
    private final ProfiledPIDController leftController = new ProfiledPIDController(1.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(leftMaxVel, leftMaxAccel));
    private final ProfiledPIDController rightController = new ProfiledPIDController(1.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(rightMaxVel, rightMaxAccel));

    //controllers is internal, look at 2021 code ;0;0 
    private double tolerance = 1.0 / 8.0; //in inches

    // goal
    private double leftGoal = 0.0;
    private double rightGoal = 0.0;

    public TelescopeArmSubsystem() {

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

    }

    @Override
    public void periodic() {

        
    
    }

    //in linear with inches
    public double getLeftEncoderValue() {
        return leftMotor.getSelectedSensorPosition() / 4096.0 * ClimberConstants.linearConversion;
    } 

    //in linear with inches
    public double getRightEncoderValue() {
        return rightMotor.getSelectedSensorPosition() / 4096.0 * ClimberConstants.linearConversion;
    }

    //in linear with inches/s
    public double getLeftVelocity() {
        // * 10 converts from units per 100ms to units per 1s
        return leftMotor.getSelectedSensorVelocity() * 10 / 4096.0 * Math.PI * 2
            * ClimberConstants.linearConversion;
    }

    //in linear with inches/s
    public double getRightVelocity() {
        // * 10 converts from units per 100ms to units per 1s
        return rightMotor.getSelectedSensorVelocity() * 10 / 4096.0 *  Math.PI * 2 
            * ClimberConstants.linearConversion;
    }

    // Get amp value methods
    public double getLeftAmps() {
        return leftMotor.getStatorCurrent();
    } 

    public double getRightAmps() {
        return rightMotor.getStatorCurrent();
    }

    //resets the encoders
    public void resetLeftEncoder() {
        leftMotor.setSelectedSensorPosition(0);
    }

    public void resetRightEncoder() {
        rightMotor.setSelectedSensorPosition(0);
    }

    public void runLeftPercent() {
        setLeftPercent(0.25);
    }

    public void runRightPercent() {
        setRightPercent(0.25);
    }

    public void retractLeft() {
        setLeftPercent(-0.25);
    }

    public void retractRight() {
        setRightPercent(-0.25);
    }

    public void setLeftPercent(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setRightPercent(double percent) {
        rightMotor.set(ControlMode.PercentOutput, percent);
    }

    //value is passed in in inches
    public void setLeftDesiredPosition(double desiredPosition){
        leftGoal = desiredPosition;
        double output = leftController.calculate(getLeftEncoderValue(), leftGoal);

        leftMotor.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void setRightDesiredPosition(double desiredPosition){
        rightGoal = desiredPosition;
        double output = rightController.calculate(getRightEncoderValue(), rightGoal);

        rightMotor.set(TalonSRXControlMode.PercentOutput, output);
    }

    public boolean atGoal() {
        return Math.abs(getLeftEncoderValue() - leftGoal) <= tolerance &&
                Math.abs(getRightEncoderValue() - rightGoal) <= tolerance;
    }

    public boolean withinBoundaries() {
        return (getLeftEncoderValue() >= -1 &&
                getRightEncoderValue() >= -1 &&
                getLeftEncoderValue() <= ClimberConstants.maxHeight &&
                getRightEncoderValue() <= ClimberConstants.maxHeight);
    }

    public void resetControllers() {
        leftController.reset(getLeftEncoderValue());
        rightController.reset(getRightEncoderValue());
    }

}