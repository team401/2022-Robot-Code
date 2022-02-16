package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SuperstructureConstants;

public class ClimbingSubsystem extends SubsystemBase {

    //sets up our two climbing arms (each which has two motors: one who can rotate and one who can't)
    private final WPI_TalonSRX leftExtensionMotor = new WPI_TalonSRX(CANDevices.firstStageLeftMotorID);
    private final WPI_TalonSRX rightExtensionMotor = new WPI_TalonSRX(CANDevices.firstStageRightMotorID);
    private final WPI_TalonSRX leftRotationMotor = new WPI_TalonSRX(CANDevices.secondStageLeftMotorID);
    private final WPI_TalonSRX rightRotationMotor = new WPI_TalonSRX(CANDevices.secondStageRightMotorID);

    private final DutyCycleEncoder leftArmEncoder = new DutyCycleEncoder(SuperstructureConstants.leftArmEncoder);
    private final DutyCycleEncoder rightArmEncoder = new DutyCycleEncoder(SuperstructureConstants.rightArmEncoder);

    //PID Controllers
    private final PIDController leftExtensionControllerLoadBearing = new PIDController(0, 0, 0);
    private final PIDController RightExtensionControllerLoadBearing = new PIDController(0, 0, 0);

    

    public ClimbingSubsystem() {

        leftExtensionMotor.configFactoryDefault();
        rightRotationMotor.configFactoryDefault();
        rightExtensionMotor.configFactoryDefault();
        leftRotationMotor.configFactoryDefault();

        leftExtensionMotor.setNeutralMode(NeutralMode.Brake);
        rightExtensionMotor.setNeutralMode(NeutralMode.Brake);
        rightExtensionMotor.setNeutralMode(NeutralMode.Brake);
        leftRotationMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {

    }

    public double getLeftExtensionEncoderValue() {
        return leftExtensionMotor.getSelectedSensorPosition();
    } 

    public double getRightExtensionEncoderValue() {
        return rightExtensionMotor.getSelectedSensorPosition();
    }

    public double getLeftExtensionAmps() {
        return leftExtensionMotor.getStatorCurrent();
    } 

    public double getRightExtensionAmps() {
        return rightExtensionMotor.getStatorCurrent();
    }

    public void setLeftExtensionPercent(double percent) {
        leftExtensionMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setRightExtensionPercent(double percent) {
        rightExtensionMotor.set(ControlMode.PercentOutput, percent);
    }








}
