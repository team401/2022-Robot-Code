package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase{

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    public static final double maxVolt = 12.0;

    public static final double maxTurningSpeed = DriveConstants.maxDriveSpeed /
        Math.hypot(DriveConstants.trackWidth / 2.0, DriveConstants.wheelBase / 2.0);


    /*
    initialize each swerve module using the parameters of Drive Motor ID, Rotation Motor ID, Rotation Encoder 
    ID (which is the CANCoder), and the initial offset 
    */

    private final SwerveModule frontLeft =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            GearRatio.L2,
            CANDevices.frontLeftDriveMotorID,
            CANDevices.frontLeftRotationMotorID, 
            CANDevices.frontLeftRotationEncoderID, 
            DriveConstants.frontLeftAngleOffset
        );

    private final SwerveModule frontRight =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            GearRatio.L2,
            CANDevices.frontRightDriveMotorID,
            CANDevices.frontRightRotationMotorID, 
            CANDevices.frontRightRotationEncoderID, 
            DriveConstants.frontRightAngleOffset
        );

    private final SwerveModule backLeft =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            GearRatio.L2,
            CANDevices.backLeftDriveMotorID,
            CANDevices.backLeftRotationMotorID, 
            CANDevices.backLeftRotationEncoderID, 
            DriveConstants.backLeftAngleOffset
        );

    private final SwerveModule backRight =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            GearRatio.L2,
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

    private ChassisSpeeds chassisSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);

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

    }

    //runs every 20 seconds, good for updating and debugging with prints
    @Override
    public void periodic() {
        
        //update the odometry with the latest heading, speed, and angle (of each module)
        odometry.update(getHeading(), getModuleStates());

        SmartDashboard.putNumber("front left", frontLeft.getSteerAngle());

    }

    //gets the heading (way the front of the robot is pointing) based on our gyro
    public Rotation2d getHeading() {
        
        //creates an array and fills it with the yaw, pitch, and roll from gyro, before returning the yaw
        double[] ypr = new double[3];
        imu.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]);

    }

    //returns all of the states (spped and angle) of the swerve modules in an array
    public SwerveModuleState[] getModuleStates() {


        SwerveModuleState[] states = {
            new SwerveModuleState(frontLeft.getDriveVelocity(), 
                new Rotation2d(frontLeft.getSteerAngle())),
            new SwerveModuleState(frontRight.getDriveVelocity(), 
                new Rotation2d(frontRight.getSteerAngle())),
            new SwerveModuleState(backLeft.getDriveVelocity(), 
                new Rotation2d(backLeft.getSteerAngle())),
            new SwerveModuleState(backRight.getDriveVelocity(), 
                new Rotation2d(backRight.getSteerAngle()))            
        };

        return states;

    }

    /**
     * Sets the state of each of the module to the desired state (which includes a speed and an angle each
     * one should be at)
     */
    public void setModuleStates(SwerveModuleState[] states) {

        frontLeft.set(states[0].speedMetersPerSecond / DriveConstants.maxDriveSpeed 
            * maxVolt, states[0].angle.getRadians());
        frontRight.set(states[1].speedMetersPerSecond / DriveConstants.maxDriveSpeed 
            * maxVolt, states[1].angle.getRadians());
        backLeft.set(states[2].speedMetersPerSecond / DriveConstants.maxDriveSpeed 
            * maxVolt, states[2].angle.getRadians());
        backRight.set(states[3].speedMetersPerSecond / DriveConstants.maxDriveSpeed 
            * maxVolt, states[3].angle.getRadians());

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
        
        chassisSpeed = speeds;
        
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

    //resets yaw/heading on gyro
    public void resetIMU(){

        imu.setYaw(0);

    }

    /*
        We can look at this later, but I don't think we need it anymore

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
    */

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
