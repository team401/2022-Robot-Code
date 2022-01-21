package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    /**
     * Controls the intake mechanism of the robot, which runs on a 775 pro and is deployed by a pair of the
     * climbing motors currently
     * When we extend and retract intake, we will call a method to go to the down position in our climbers(?)
     */

     private final TalonSRX intakeMotor = new TalonSRX(CANDevices.intakeMotorID);


    //Constructor
    public IntakeSubsystem() {

        //should limit our intake to a specific current (just in case)
        //might not work?
        intakeMotor.configPeakCurrentLimit(40);

    }

    @Override
    public void periodic() {

        //for testing use (to see if the current limit works :))
        SmartDashboard.putNumber("intake motor measured current", intakeMotor.getSupplyCurrent());

    }

    //runs the motor in percent mode based on our constants
    public void runIntakeMotor() {

        intakeMotor.set(ControlMode.PercentOutput, SuperstructureConstants.intakingPower);

    }

    //runs the motor in the opposite direction the same amount
    public void reverseIntakeMotor() {

        intakeMotor.set(ControlMode.PercentOutput, -SuperstructureConstants.intakingPower);

    }

    //stops the motor by setting it to 0
    public void stopIntakeMotor() {

        intakeMotor.set(ControlMode.PercentOutput, 0.0);

    }

    //methods that are set up right now to do nothing, but will default to the climber arms (?)
    public void extendIntake() {
        //down position of climbing rotation motor
    }

    public void retractIntake() {
        //up position of climbing rotation motor
    }

    public void intakeState() {
        //gets the position the climbing rotation motor
    }

}
