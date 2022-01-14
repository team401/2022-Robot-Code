package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase{

    /*
    initialize each swerve module using the parameters of Drive Motor ID, Rotation Motor ID, Rotation Encoder 
    ID (which is the CANCoder), and the initial offset 
    */
    
    private final SwerveModule frontLeft =
        new SwerveModule(
            CANDevices.frontLeftDriveMotorID,
            CANDevices.frontLeftRotationMotorID,
            CANDevices.frontLeftRotationEncoderID,
            DriveConstants.frontLeftAngleOffset
        );

    private final SwerveModule frontRight =
        new SwerveModule(
            CANDevices.frontRightDriveMotorID,
            CANDevices.frontRightRotationMotorID,
            CANDevices.frontRightRotationEncoderID,
            DriveConstants.frontRightAngleOffset
        );

    private final SwerveModule backLeft =
        new SwerveModule(
            CANDevices.backLeftDriveMotorID,
            CANDevices.backLeftRotationMotorID,
            CANDevices.backLeftRotationEncoderID,
            DriveConstants.backLeftAngleOffset
        );    

    private final SwerveModule backRight =
        new SwerveModule(
            CANDevices.backRightDriveMotorID,
            CANDevices.backRightRotationMotorID,
            CANDevices.backRightRotationEncoderID,
            DriveConstants.backRightAngleOffset
        );

    /*
    determines whether we are using field relative inputs or not (true: pushing forward moves the robot forward
    relative to the driver, false: pushing forward moves the robot forward relative to the front of the robot)
    */

    private boolean isCommandFieldRelative = false;

    //initialize gyro
    private final PigeonIMU imu = new PigeonIMU(CANDevices.imuID);

    /**
     * Odometry model for our serve drive
     * Linear: in meters
     * Rotation: in radians
     * Basedd on the kinematics and our overall angle
     */
    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(
            DriveConstants.kinematics,
            new Rotation2d(getHeading().getRadians()
        )
    );

    //all that is done in the constructor is initialize offsets, reset imu, and set distance traveled to 0
    public DriveSubsystem() {
        
        resetIMU();

        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        backLeft.initRotationOffset();
        backRight.initRotationOffset();

        frontLeft.resetDistance();
        frontRight.resetDistance();
        backLeft.resetDistance();
        backRight.resetDistance();

    }

    //gets the heading (way the front of the robot is pointing) based on our gyro
    public Rotation2d getHeading() {
        
        //creates an array and fills it with the yaw, pitch, and roll from gyro, before returning the yaw
        double[] ypr = new double[3];
        imu.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]);

    }

    //resets yaw/heading on gyro
    public void resetIMU(){

        imu.setYaw(0);

    }

    //returns all of the states (spped and angle) of the swerve modules in an array
    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getCanEncoderAngle()),
            new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getCanEncoderAngle()),
            new SwerveModuleState(backLeft.getCurrentVelocityMetersPerSecond(), backLeft.getCanEncoderAngle()),
            new SwerveModuleState(backRight.getCurrentVelocityMetersPerSecond(), backRight.getCanEncoderAngle())            
        };

        return states;

    }

    //runs every 20 seconds, good for updating and debugging with prints
    @Override
    public void periodic() {

        //update the odometry with the latest heading, speed, and angle (of each module)
        odometry.update(getHeading(), getModuleStates());

    }

    //commanding drive
    public void drive() {



    }
    
}
