package frc.robot.subsystems;


import java.rmi.Remote;

import javax.sql.rowset.RowSetFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
    
    //dual Falcon 500s on each of our swerve modules
    private final WPI_TalonFX driveMotor;       
    private final WPI_TalonFX rotationMotor;

    private final CANCoder canCoder; //Absolute Encoder

    private final Rotation2d offset; //So wheels can be aligned when robot is turned on

    //creating the two PID controllers using the values in our constant folder
    private final PIDController driveController = new PIDController(
        DriveConstants.drivekP, DriveConstants.drivekI, DriveConstants.drivekD);

    //value we are sending to the PID
    private double desiredClosedLoopTargetAngle = 0.0;

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

        rotationMotor.configFactoryDefault();

        //initializing CANCoder  to be from 0 to 360 and set up Offset Rotation2d for the rotation motor
        canCoder = new CANCoder(cancoderID);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        offset = new Rotation2d(measuredOffsetRadians);

        //setting idle modes of the two motors
        driveMotor.setNeutralMode(NeutralMode.Coast);
        rotationMotor.setNeutralMode(NeutralMode.Coast);

        //configurations for our rotation motors
        rotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 10);
        rotationMotor.setSensorPhase(true);
        rotationMotor.selectProfileSlot(0, 0);
        //rotationMotor.setInverted(true);

        //****CHANGE ME FOR TESTING****
        //sets the PID values for position in the rotation motor 
        //untested, might work?
        rotationMotor.configAllowableClosedloopError(0, 0, 30);
        rotationMotor.config_kP(0, 0.5);
        rotationMotor.config_kI(0, 0);
        rotationMotor.config_kD(0, 0);
        rotationMotor.configRemoteFeedbackFilter(cancoderID, RemoteSensorSource.CANCoder, 0);
        
        //CanCoder config
        //canCoder.configMagnetOffset(-offset.getDegrees()); //TODO: Figure out how this works

    }

    //sets traveled distance in internal encoder to 0
    public void resetDistance() {
        
        driveMotor.setSelectedSensorPosition(0.0);
    
    }

    //returns the current encoder position in radians by multiplying from conversion factor
    public double getDriveDistanceRadians() {

        return driveMotor.getSelectedSensorPosition() * DriveConstants.driveSensorConversionFactor;

    }

    //sets drive motor to run at a percent for basic testing/control
    public void setDrivePercent(double percent) {

        driveMotor.set(percent);

    }

    //sets rotation motor to run at a percent for basic testing/control
    public void setRotationPercent(double percent){

        rotationMotor.set(percent);

    }

    //sends rotation with position of CanCoder angle pointing (note - if returns negative, add in if case)
    public Rotation2d getCanCoderAngle() {

        return new Rotation2d(
            //(Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians()) % ( 2 * Math.PI ));
            (Units.degreesToRadians(canCoder.getAbsolutePosition())) % ( 2 * Math.PI ));
    }
    
    //**CHECK ME***
    //gets angle from the internal encoder in the rotation motor 
    public Rotation2d getInternalRotationAngle() {

        //gear reduction is needed, and we always have to divide by the # of tics in a rotation to get raidans
        return new Rotation2d((
            rotationMotor.getSelectedSensorPosition() / DriveConstants.rotationWheelGearReduction
            / 2048.0 * 2 * Math.PI) % (2 * Math.PI));

    }

    /*public Rotation2d setInternalRotationAngle() {

        rotationMotor.setSelectedSensorPosition(getCanCoderAngle())

    }*/

    //***CHECK ME***
    //just a test to make sure that the gear reduction is right
    public double getInternalRotationAngleTest() {

        return (rotationMotor.getSelectedSensorPosition() / (150.0/7.0));

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
    public void initRotation() {

        rotationMotor.setSelectedSensorPosition(
            getCanCoderAngle().getRadians() / DriveConstants.FalconSensorConversionFactor);

        

    }

    //will send the desired position off the PID loop in degrees
    public Rotation2d getPositionPIDValue() {

        return new Rotation2d(desiredClosedLoopTargetAngle);

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

        //***NEW SECTION***
        //sends the calculated position
        //need to convert from radians to tics/sensor position

        /*desiredClosedLoopTargetAngle = calculateAdjustedAngle(
            state.angle.getRadians(), 
            getCanCoderAngle().getRadians());*/ //TODO: Fix this

        desiredClosedLoopTargetAngle = calculateAdjustedAngle(
            state.angle.getRadians(), 
            getInternalRotationAngle().getRadians());

        rotationMotor.set(TalonFXControlMode.Position, 
            desiredClosedLoopTargetAngle / (2 * Math.PI) * 4096
            );

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
        driveMotor.setVoltage(driveVelocityFFCalculated + driveVelocityPIDCalculated);
        
    }

}
