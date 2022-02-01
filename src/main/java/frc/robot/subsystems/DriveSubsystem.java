package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    //Variables to store our current commanded values to the robot
    private double commandedForward = 0.0;
    private double commandedStrafe = 0.0;
    private double commandedRotation = 0.0;

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

    //runs every 20 seconds, good for updating and debugging with prints
    @Override
    public void periodic() {
        
        //update the odometry with the latest heading, speed, and angle (of each module)
        odometry.update(getHeading(), getModuleStates());

        

        SmartDashboard.putNumber("frontLeft Measured", frontLeft.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("frontRight Measured", frontRight.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("backLeft Measured", backLeft.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("backRight Measured", backRight.getCanCoderAngle().getDegrees());

        SmartDashboard.putNumber("Rotation Desirec", commandedRotation);

        /*SmartDashboard.putBoolean("frontLeft Within Tolerance", Math.abs(frontLeft.getCanCoderAngle().getDegrees()) < 0.5);
        SmartDashboard.putBoolean("frontRight Within Tolerance", Math.abs(frontRight.getCanCoderAngle().getDegrees()-180) < 0.5);
        SmartDashboard.putBoolean("backLeft Within Tolerance", Math.abs(backLeft.getCanCoderAngle().getDegrees()) < 0.5);
        SmartDashboard.putBoolean("backRight Within Tolerance", Math.abs(backRight.getCanCoderAngle().getDegrees()-180) < 0.5);

        SmartDashboard.putNumber("frontLeft Measured Drive", frontLeft.getDriveDistanceRadians() / (2 * Math.PI));
        SmartDashboard.putNumber("frontRight Measured Drive", frontRight.getDriveDistanceRadians()/ (2 * Math.PI));
        SmartDashboard.putNumber("backLeft Measured Drive", backLeft.getDriveDistanceRadians()/ (2 * Math.PI));
        SmartDashboard.putNumber("backRight Measured Drive", backRight.getDriveDistanceRadians()/ (2 * Math.PI));

        SmartDashboard.putNumber("Average Measured Drive", 
            getAverageDriveDistanceRadians() / (2 * Math.PI)
        );

        //some useful prints that will be added to SmartDashboard for premptive debugging
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Odometry x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry y", odometry.getPoseMeters().getY());*/
        

    }

    //gets the heading (way the front of the robot is pointing) based on our gyro
    public Rotation2d getHeading() {
        
        //creates an array and fills it with the yaw, pitch, and roll from gyro, before returning the yaw
        double[] ypr = new double[3];
        imu.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]);

    }

    //testing percent
    public void runTestatPercent() {

        frontRight.setDrivePercent(0.1);
        frontLeft.setDrivePercent(0.1);
        backLeft.setDrivePercent(0.1);
        backRight.setDrivePercent(0.1);

    }

    //testing rotation
    public void runTestatPercentSpin() {

        frontRight.setRotationPercent(0.1);
        frontLeft.setRotationPercent(0.1);
        backLeft.setRotationPercent(0.1);
        backRight.setRotationPercent(0.1);

    }

    //stops the motors
    public void stopDriving() {

        frontRight.setDrivePercent(0.0);
        frontLeft.setDrivePercent(0.0);
        backLeft.setDrivePercent(0.0);
        backRight.setDrivePercent(0.0);
        frontRight.setRotationPercent(0.0);
        frontLeft.setRotationPercent(0.0);
        backLeft.setRotationPercent(0.0);
        backRight.setRotationPercent(0.0);
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

    /**
     * Sets the state of each of the module to the desired state (which includes a speed and an angle each
     * one should be at)
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
        frontRight.setDesiredStateClosedLoop(moduleStates[1]);
        backLeft.setDesiredStateClosedLoop(moduleStates[2]);
        backRight.setDesiredStateClosedLoop(moduleStates[3]);

    }

    /**
     * Drive Method which takes in:
     * Forward Linear
     * Sideways Linear
     * Rotation
     * Whether it is field relative or not
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        //updates what values we are currently giving to the robot
        commandedForward = forward;
        commandedStrafe = strafe;
        commandedRotation = rotation;
        isCommandFieldRelative = isFieldRelative;

        /** Creates a ChassisSpeeds Object, which represents the overall desired movement of the robot
         * Based on the three speed values we give, it'll create states we can feed to set the module
         * states of the swerve drive
         * Also, the angle will be determined based on whether or not it is field relative, which is what
         * the ? symbolizes (lamda ? statement)
         */
        ChassisSpeeds speeds = 
            isFieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, getHeading())
                : new ChassisSpeeds(forward, strafe, rotation);
        
        //based off of the kinematics, it'll convert it to Swerve Module States
        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);

        //ensure that we do not go over the max speed of the robot
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeed);

        SmartDashboard.putNumber("front left command rot", states[0].angle.getDegrees());

        //sets the states of the modules to the desired, commanded ones
        setModuleStates(states);

    }
    
    //returns the current position of the robot on field based on drive encoders and gyro reading
    public Pose2d getPose() {

        return odometry.getPoseMeters();

    }

    //resets our current pose to one that is given
    public void resetPose(Pose2d pose) {

        imu.setYaw(0); //might not be needed?
        odometry.resetPosition(pose, getHeading());

    }

    //resets the measured drive distance in each module (to 0)
    public void resetDriveDistance() {

        frontLeft.resetDistance();
        frontRight.resetDistance();
        backLeft.resetDistance();
        backRight.resetDistance();

    }

    //resets yaw/heading on gyro
    public void resetIMU(){

        imu.setYaw(0);

    }

    //finds the mean distance driven by each module in radians to get an estimae of the 
    //overall distance driven
    public double getAverageDriveDistanceRadians() {

        return ((
            Math.abs(frontLeft.getDriveDistanceRadians()) +
            Math.abs(frontRight.getDriveDistanceRadians()) +
            Math.abs(backLeft.getDriveDistanceRadians()) +
            Math.abs(backRight.getDriveDistanceRadians())) / 4.0
        );

    }

    //finds the mean velocity of the swerve modules to estimate an overall velocity for the robot
    public double getAverageDriveVelocityRadiansPerSecond() {

        return ((
            Math.abs(frontLeft.getCurrentVelocityRadiansPerSecond()) +
            Math.abs(frontRight.getCurrentVelocityRadiansPerSecond()) +
            Math.abs(backLeft.getCurrentVelocityRadiansPerSecond()) +
            Math.abs(backRight.getCurrentVelocityRadiansPerSecond())) / 4.0
        );

    }

    //returns an array of all of the current values we are commanding to robot to drive
    public double[] getCommandedDriveValues() {

        double[] values = {commandedForward, commandedStrafe, commandedRotation};

        return values;

    }

    //returns whether or not our current commanded drive values are field relative or not
    public boolean getIsFieldRelative() {

        return isCommandFieldRelative;

    }

}
