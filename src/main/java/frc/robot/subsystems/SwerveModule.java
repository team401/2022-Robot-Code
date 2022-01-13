package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
    
    private static final double drivekP = 0; //Characterization takes care of some error already

    //dual Falcon 500s on each of our swerve modules
    private final WPI_TalonFX driveMotor;       
    private final WPI_TalonFX rotationMotor;

    private final CANCoder canCoder; //Absolute Encoder

    private final Rotation2d offset; //So wheels can be aligned when robot is turned on

    //creating the two PID controllers using the values in our constant folder
    private final PIDController driveController = new PIDController(
        DriveConstants.drivekP, DriveConstants.drivekI, DriveConstants.drivekD);
    private final PIDController rotationController = new PIDController(
        DriveConstants.rotationkP, DriveConstants.rotationkI, DriveConstants.rotationkD);

    //Constructor
    public SwerveModule(
        int driveMotorID,
        int rotationMotorID,
        int cancoderID,
        double measuredOffsetRadians
    ) {
        //initializing motors
        driveMotor = new WPI_TalonFX(driveMotorID);
        rotationMotor = new WPI_TalonFX(rotationMotorID);

        //initializing CANCoder  to be from 0 to 360 and set up Offset Rotation2d for the rotation motor
        canCoder = new CANCoder(cancoderID);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        offset = new Rotation2d(measuredOffsetRadians);

        //setting idle modes of the two motors
        driveMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.setNeutralMode(NeutralMode.Coast);
    }

    //sets traveled distance in internal encoder to 0
    public void resetDistance() {
        
        driveMotor.setSelectedSensorPosition(0.0);
    
    }

    //returns the current encoder position in radians by multiplying from conversion factor
    public double getDriveDistanceRadians() {

        return driveMotor.getSelectedSensorPosition() * DriveConstants.driveSensorConversionFactor;

    }

    //sends rotation with position of CanCoder angle pointing (note - if returns negative, add in if case)
    public Rotation2d getCanCoderAngle() {

        return new Rotation2d(
            (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians()) % (2 * Math.PI));
   
    }

    //gets angle from the internal encoder in the rotation motor 
    public Rotation2d getCanEncoderAngle() {

        double unsignedAngle = 
            (rotationMotor.getSelectedSensorPosition() * DriveConstants.FalconSensorConversionFactor) % (2 * Math.PI);
        
        //fail-safe just in case the unsigned angle is negative
        if (unsignedAngle < 0) unsignedAngle += 2 * Math.PI;

        return new Rotation2d(unsignedAngle);

    }

    //gets current velocity of the drive motor in rads/sec
    public double getCurrentVelocityRadiansPerSecond() {

        return driveMotor.getSelectedSensorVelocity() * DriveConstants.driveSensorConversionFactor;

    }

    //gets current velocity of the drive motor in m/s
    public double getCurrentVelocityMetersPerSecond() {

        return driveMotor.getSelectedSensorVelocity() * DriveConstants.driveSensorConversionFactor 
            * (DriveConstants.wheelDiameterMeters / 2.0);

    }

    //calculating the setpoint we will feed based on the desired angle and the current angle
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        //Finds the current angle in a range of [0, 2pi]
        double modAngle = currentAngle % (2.0 * Math.PI);
        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;

        /*
        finds the adjusted target angle by increasing the value of the target angle 
        (which is between 0 and 2pi) by the number of radians in the difference between 
        current angle and modangle (which is equivalent to the number of full rotations
        the motor has made around in degrees, which will always be multiples of 2pi)
        */
        double newTarget = targetAngle + (currentAngle - modAngle);

        //Optimizes the angle so make sure the wheel will never have to turn more than half a rotation
        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;

    }

    //initializes the CANCoder to the offset measurements from its current reading
    public void initRotationOffset() {

        canCoder.setPosition(getCanCoderAngle().getRadians());

    }

    /**
    ***WIP***
    sets the reference in a closed loop in both the drive motor (using velocity as the control and FF as our 
    main model, with PID as a supplement) and the rotation motor (using position as the control and all done
    with the PID controller)
    EXPERIMENTAL
    */
    public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {

        //takes in the state
        SwerveModuleState state = desiredState;
        
        /*
        finds the position we need to send to the rotation motor using our PID controller
        by using the current measured position and what the passed in state wants (with adjustment)
        all in radians
        */
        double rotationPosition = rotationController.calculate(
            rotationMotor.getSelectedSensorPosition() * DriveConstants.FalconSensorConversionFactor,
            calculateAdjustedAngle(
                state.angle.getRadians(), 
                getCanEncoderAngle().getRadians())
        );

        //sends the calculated position value to the rotation motor
        rotationMotor.set(ControlMode.Position, rotationPosition/DriveConstants.FalconSensorConversionFactor);

        //calculates drive speed of the modules
        double speedRadPerSec = state.speedMetersPerSecond / (DriveConstants.wheelDiameterMeters / 2);
                
        //calculate what our current speed should be using the FeedForward model
        double driveVelocityFFCalculated = DriveConstants.driveFF.calculate(speedRadPerSec);

        //calculates what we should add to our current speed using a PID control model
        double driveVelocityPIDCalculated = 
            driveController.calculate(
                speedRadPerSec,
                getCurrentVelocityRadiansPerSecond()
            );

        //sends the sum of the two calculated velocities to the drive motor
        driveMotor.set(ControlMode.Velocity, driveVelocityFFCalculated + driveVelocityPIDCalculated);
        
    }


}
