package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
    
    //dual Falcon 500s on each of our swerve modules
    private final WPI_TalonFX driveMotor;       
    private final WPI_TalonFX rotationMotor;

    private final CANCoder canCoder; //Absolute Encoder

    private final Rotation2d offset; //So wheels can be aligned when robot is turned on

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

        //resets the rotation motor to the default settings
        rotationMotor.configFactoryDefault();

        //initializing CANCoder  to be from 0 to 360 and set up Offset Rotation2d for the rotation motor
        canCoder = new CANCoder(cancoderID);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); //maybe don't need this line?
        offset = new Rotation2d(measuredOffsetRadians);

        //resets rotation motor and cancoder to default settings
        rotationMotor.configFactoryDefault();
        canCoder.configFactoryDefault();

        //setting idle modes of the two motors
        driveMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.setNeutralMode(NeutralMode.Coast);

        //configurations for our rotation motors
        rotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 10);
        rotationMotor.setSensorPhase(true);
        rotationMotor.selectProfileSlot(0, 0);
        rotationMotor.setInverted(true);

        //****CHANGE ME FOR TESTING****
        //sets the PID values for position in the rotation motor 
        //untested, might work?
        rotationMotor.configAllowableClosedloopError(0, 0, 30);
        rotationMotor.config_kP(0, DriveConstants.rotationkP);
        rotationMotor.config_kI(0, DriveConstants.rotationkI);
        rotationMotor.config_kD(0, DriveConstants.rotationkD);
        rotationMotor.configRemoteFeedbackFilter(cancoderID, RemoteSensorSource.CANCoder, 0);
        rotationMotor.configFeedbackNotContinuous(false, 0);
        
        //CanCoder config
        canCoder.configMagnetOffset(-offset.getDegrees());
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

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
            (Units.degreesToRadians(canCoder.getPosition())));

    }
    
    //**CHECK ME***
    //gets angle from the internal encoder in the rotation motor 
    public Rotation2d getInternalRotationAngle() {

        //gear reduction is needed, and we always have to divide by the # of tics in a rotation to get raidans
        return new Rotation2d((
            rotationMotor.getSelectedSensorPosition() / DriveConstants.rotationWheelGearReduction
            / 2048.0 * 2 * Math.PI) % (2 * Math.PI));

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
    sets the reference in a closed loop in both the drive motor (using velocity as the control with FF) 
    and the rotation motor (using position as the control and all done
    with the PID controller)
    */
    public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {

        //takes in the state
        SwerveModuleState state = desiredState;

        //***NEW SECTION***
        //sends the calculated position by optimizing it
        //need to convert from radians to tics/sensor position
        state = optimize(state, getCanCoderAngle());
        rotationMotor.set(TalonFXControlMode.Position,  state.angle.getRadians() / (2 * Math.PI) * 4096);

        //calculates drive speed of the modules
        double speedRadPerSec = state.speedMetersPerSecond / (DriveConstants.wheelDiameterMeters / 2);
                
        //calculate what our current speed should be using the FeedForward model
        double driveVelocityFFCalculated = DriveConstants.driveFF.calculate(speedRadPerSec);

        //sends the sum of the two calculated velocities to the drive motor
        driveMotor.setVoltage(driveVelocityFFCalculated);
        
    }

    //ALL CREDIT TO 364 <- The literal best <3
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double lowerBound;
        double upperBound;

        double scopeReference = currentAngle.getDegrees();
        double newAngle = desiredState.angle.getDegrees();
        double lowerOffset = scopeReference % 360;

        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

        while (newAngle < lowerBound) {
            newAngle += 360;
        }

        while (newAngle > upperBound) {
            newAngle -= 360;
        }

        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = newAngle - currentAngle.getDegrees();
        
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            newAngle = delta > 90 ? (newAngle -= 180) : (newAngle += 180);
        }        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(newAngle));
    }

}
