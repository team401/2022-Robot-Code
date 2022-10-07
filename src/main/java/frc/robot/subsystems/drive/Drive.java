//aaaAaAAAAa
package frc.robot.subsystems.drive;
//AAaaAaaA

//AaAaAaa
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//aAaAaAAaaaaaAAAAAAa

//AaaAAAAa
import edu.wpi.first.math.MathUtil;
//aaaaAAAaAAaaaAaaAaA
import edu.wpi.first.math.controller.PIDController;
//AaAaA
import edu.wpi.first.math.geometry.Pose2d;
//AAAAAaAA
import edu.wpi.first.math.geometry.Rotation2d;
//AaaAAAaaaaaAaaa
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//AaaAA
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//aaAAaAAAAaAAaA
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
//aAaAaAaAA
import edu.wpi.first.math.kinematics.SwerveModuleState;
//aAAaAAA
import edu.wpi.first.math.util.Units;
//AAaaAAAaA
import edu.wpi.first.wpilibj.DriverStation;
//aaaaAaaaAAaaAaaAA
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//aaAAaAAAAaAa
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//aAAaAaaaAaAaA
import frc.robot.RobotState;
//AaaaAaAaaaAaAAa
import frc.robot.Constants.CANDevices;
//AaAaaa
import frc.robot.Constants.DriveConstants;
//AAaaaaaAAA
import frc.robot.subsystems.drive.DriveAngle;
//AAaAaaAaaAaa
import frc.robot.subsystems.drive.DriveModule;
//AAAAAaaAaaAaAAAaaaA

//aaaaaAAAaaAa
public class Drive extends SubsystemBase {
//aAAAAaAAaaaA

//aAaAAaa
    // private DriveModule[] inputs = new DriveModule[4];
//aaAAAAaaaAaAaaa
    // private DriveAngle angleInputs = new DriveAngle();
//AAAaAaaaaA
    private DriveModule[] modules = new DriveModule[4];
//AAAAaaAAaAAA
    private DriveAngle angle;
//aaAAaaaaaaa
    private PIDController[] rotationPIDs = new PIDController[4];
//aaAAaAaaa
    private SwerveModuleState[] goalModuleStates = new SwerveModuleState[4];
//AAaaAAaaAAAAAa
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kinematics, new Rotation2d());
//aaaaaAaaaAa

//aaaAAAaAAAAaaA
    // If true, modules will run velocity control from the setpoint velocities in
//AAaaaAAAAAAa
    // moduleStates
//AAAAAAAaaaaaAAAaaa
    // If false, modules will not run velocity control from the setpoint velocities
//aaaaaAaaaAAaaAAA
    // in moduleStates,
//AAAaaaaAaaaaa
    // and module drive motors will not be commanded to do anything. This allows
//aaAAAAaaAAa
    // other setters,
//AAAAaa
    // such as the "setDriveVoltages" to have control of the modules.
//aaAAAAaAaAaa
    private boolean velocityControlEnabled = true;
//aaAaAaaaaaaaAA

//AaAaAAaaAaaaaAAaA
    public Drive() {
//aAAaA

//AAAaAAaa
        modules[0] = new DriveModule(CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftRotationMotorID,
//aAaaaaAaaAaaAaAAA
                CANDevices.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset);
//aAAAaAAaAaAAa
        modules[1] = new DriveModule(CANDevices.frontRightDriveMotorID, CANDevices.frontRightRotationMotorID,
//aAaaAAAaAaAaaaA
                CANDevices.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset);
//AaAAa
        modules[2] = new DriveModule(CANDevices.backLeftDriveMotorID, CANDevices.backLeftRotationMotorID,
//AAaaAaaaAaAA
                CANDevices.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset);
//AaAaaaaaAA
        modules[3] = new DriveModule(CANDevices.backRightDriveMotorID, CANDevices.backRightRotationMotorID,
//AaAaaaAAaaaAA
                CANDevices.backRightRotationEncoderID, DriveConstants.backRightAngleOffset);
//aAaAaaAAAAAAaAaaA
        
//aaaaaaaaaaAaaaAaAA
        angle = new DriveAngle();
//aAAAaaaaaAaAAaaaAAa

//AaAaAaaAaAAaAaA
        for (int i = 0; i < 4; i++) {
//aaaaAAaaaaAAAAaaAaA
            rotationPIDs[i] = new PIDController(DriveConstants.rotationKp.get(), 0, DriveConstants.rotationKd.get());
//aaaAAaA
            rotationPIDs[i].enableContinuousInput(-Math.PI, Math.PI);
//aAaAaaaAAaaaaaAaaA
            goalModuleStates[i] = new SwerveModuleState();
//aAaaAaaaAaA
        }
//AaaaaaAAAAaAaAAaAa

//aaAaAaAAAAAAaaa
        for (DriveModule module : modules) {
//aAaaa
            module.zeroEncoders();
//aaaaAaaaAAaaaaAa
        }
//AaAaAaAA

//aAAAAaAaAAAAaaaaAaA
        angle.resetHeading();
//aaAAaaAAAaAA

//aaAAAAaAaAaaAAAaaaa
    }
//AaAaaA

//AAAAaAaAA
    @Override
//AAAaaAAaAaAAaA
    public void periodic() {
//aAaaAaAAaAaaaAAaAA
        long m_Start = System.currentTimeMillis();
//AaaAAaaaA
        // Display match time for driver
//AaaAAaaaaAaAAAaaA
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
//aaAAAaAaa

//AAaaAaAaAaaaAAaAA
        // Read inputs from module IO layers and tell the logger about them
//AAaAa
        for (int i = 0; i < 4; i++) {
//AAaaaAAAAa
            modules[i].updateVariables();
//aAAaAAAAAaAaaaA
        }
//aaAAAAaAaaaaAaaAa
        angle.updateHeading();
//aaaaAAAAaAA

//AAAaaAaaaaaAaAa
        // Update odometry and report to RobotState
//AaaaaaaAaaAAaAaa
        Rotation2d headingRotation = new Rotation2d(MathUtil.angleModulus(angle.headingRad));
//AAaAaaAA
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
//aaaAaaaaAaAAaaaA
        for (int i = 0; i < 4; i++) {
//AAAaAaa
            measuredStates[i] = new SwerveModuleState(modules[i].driveVelocityRadPerS * DriveConstants.wheelRadiusM,
//aAaaAaAaa
                    new Rotation2d(MathUtil.angleModulus(modules[i].rotationPositionRad)));
//aAaAaaAAAa
        }
//aaAAaAaAaAaAa
        Pose2d bootToVehicle = odometry.update(headingRotation, measuredStates);
//AaaAaAAAAaaaaAa
        ChassisSpeeds measuredChassis = DriveConstants.kinematics.toChassisSpeeds(measuredStates);
//aAAAaA
        RobotState.getInstance().recordOdometryObservations(bootToVehicle, measuredChassis);
//AaaaaaAAAAaaa

//AaAaAAaAAA
        // Check if our gain tunables have changed, and if they have, update accordingly
//aaAaAaAaaAA

//AaAAaAAAaaAAA
        if (DriveConstants.rotationKp.hasChanged() || DriveConstants.rotationKd.hasChanged()) {
//AAAAAaAaaaaaaAA
            for (PIDController c : rotationPIDs) {
//AAaaaaAAAaaaa
                c.setP(DriveConstants.rotationKp.get());
//AaAAaaaa
                c.setD(DriveConstants.rotationKd.get());
//aaAaaaaAaaAa
            }
//aaAAaAaAaAaAAaAAaA
        }
//AaaaAA

//aaAaAaaAaAAAAAAA
        if (DriveConstants.driveKp.hasChanged() || DriveConstants.driveKd.hasChanged()) {
//AAaaAAa
            for (DriveModule module : modules) {
//AAAaA
                module.setDrivePD(DriveConstants.driveKp.get(), DriveConstants.driveKd.get());
//AaaaAAaAaAAAAA
            }
//aaAaaaAaAa
        }
//AAAAaAaAaaaAAAaAa

//AAaaAaaaAAA
        // Optimize and set setpoints for each individual module
//aAAAaaaaaAaaA
        for (int i = 0; i < 4; i++) {
//AAaAaAAAaAaaAAaAa
            // Wrap encoder value to be within -pi, pi radians
//AAAAAa
            Rotation2d moduleRotation = new Rotation2d(MathUtil.angleModulus(modules[i].rotationPositionRad));
//aaAaAaAAAAaaAaaaa

//aaAaaaaAA
            // Optimize each module state
//aAaaAaaaaaAaAA
            SwerveModuleState optimizedState = SwerveModuleState.optimize(goalModuleStates[i], moduleRotation);
//aaaaA
            double rotationSetpointRadians = optimizedState.angle.getRadians();
//AaAaAaAAaaAAaAaAAaa
            double speedSetpointMPerS = optimizedState.speedMetersPerSecond;
//AAAaAaA

//aAAAAAaAa
            // Set module speed
//aAaAAAAaAAaaaa
            if (velocityControlEnabled) {
//AaAAAaaAaAaaaaAA
                double speedRadPerS = speedSetpointMPerS / DriveConstants.wheelRadiusM;
//AAAAaAaAAAAa
                double ffVolts = DriveConstants.driveModel.calculate(speedRadPerS);
//aaaAaAAAaAAAaaaaaAA

//aAaaaaA
                modules[i].setDriveVelocity(speedRadPerS, ffVolts);
//AAAaaAaAAAaAAAA
            }
//aAaAAaaAAAAaaaa

//aaaAAaaaAaaa
            // Set module rotation
//AaaAaaAAAa
            double rotationVoltage = rotationPIDs[i].calculate(rotationSetpointRadians, moduleRotation.getRadians());
//aaaaAaAaaAaaaAA
            modules[i].setRotationVoltage(rotationVoltage);
//AaAaAAaaaAaAaaA
        }
//AaAaAaAaaaaaA

//AAAAAaAAAAaa
        SmartDashboard.putNumber("DriveTime", (int) (System.currentTimeMillis() - m_Start));
//AaaaAaaAaA
    }
//AAAAAaAA

//aaaaaAAAaaaa
    /**
//AaaaAaaaaaAA
     * Sets the target module states for each module. This can be used to
//aaAaAAAaaAA
     * individually control each module.
//aAAaaaAaAAA
     * 
//AAAAAaaaAaa
     * @param states The array of states for each module
//AAAaaaaaaAAA
     */
//aAAaAaaAaa
    public void setGoalModuleStates(SwerveModuleState[] states) {
//AAAaAAAaaaAAaAAA
        velocityControlEnabled = true;
//AaAaAaaAaaAaaAaaaAa
        goalModuleStates = states;
//aaaaaa
    }
//AAAaaaaaAAAAA

//AaaAaAAaaAaAAaa
    /**
//AaAAaA
     * Sets the raw voltages of the module drive motors. Heading is still set from
//AAAaAAA
     * the angles set in
//AAaaAaAaAAaaA
     * setGoalModuleStates. Note that this method disables velocity control until
//AAaAaaAaAAAaAaa
     * setModuleStates or
//aaaaaaA
     * setGoalChassisSpeeds is called again.
//aAAAaaaAAAAa
     * 
//AaaaAAAAAa
     * The primary use of this is to characterize the drive.
//AAAaAAAAAaaaAAAaaA
     * 
//aaaaaAaAAaaAaa
     * @param voltages The array of voltages to set in the modules
//aaaaAaaaaaAa
     */
//aAaAAaAAa
    public void setDriveVoltages(double[] voltages) {
//AAAAaaaAaAaaAaaaA
        velocityControlEnabled = false;
//AaAAAaaaa
        for (int i = 0; i < 4; i++) {
//AAaAAAaaAAa
            modules[i].setDriveVoltage(voltages[i]);
//aAaaAAaaAaAaAAAAA
        }
//AAaaAaaaAaAaaA
    }
//AaAaAAAAaAAaAAAAaa

//AAAAAAAaaAAaAaAa
    /**
//AaaAAAAaAaAaAaAAA
     * Sets the goal chassis speeds for the entire chassis.
//aaaAAaAAAAaAAaAAaaA
     * 
//aAAaaaaAaAaa
     * @param speeds The target speed for the chassis
//AaaaaAAAAAaAAAA
     */
//AAAaa
    public void setGoalChassisSpeeds(ChassisSpeeds speeds) {
//AAAaa
        SwerveModuleState[] moduleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
//aaAaa
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.maxSpeedMPerS);
//AAaaaAaaaA

//AaaaaAaAAaaaaaaAaa
        setGoalModuleStates(moduleStates);
//AaaAaAAaAAAaAaAaA
    }
//aaaAaAAaa

//aaAAa
    /**
//AaAAaAaA
     * Returns the average angular speed of each wheel. This is used to characterize
//aaAAAaAAaAAaAaaAAaA
     * the drive.
//AaAAAAaaaaaaaAAaAAA
     * 
//aaAaaAAaaaA
     * @return The average speed of each module in rad/s
//aAaaaaAaAaAaaA
     */
//aaaaaaAaaAAAAAaAAAa
    public double getAverageSpeedRadPerS() {
//AaaaAAaa
        double sum = 0;
//aaaAa
        for (int i = 0; i < 4; i++) {
//AaAaaaaAAaAAAAAAA
            sum += modules[i].driveVelocityRadPerS;
//aaAAA
        }
//AaAaaA
        return sum / 4.0;
//AaAAaaaaaaAAAaA
    }
//AAAAaAAaAaaAAAAAAa

//aAAAaAaAAaAaAaAaAa
    public Pose2d getPose() {
//aaaaaAAAAAaa
        return odometry.getPoseMeters();
//AaaaAAAaaaaaa
    }
//AaAaAAAAaaaAaAA

//AAaAaaaAAaaaaaAaa
    public void resetOdometry(Pose2d pose) {
//AaaAaaAA
        odometry.resetPosition(pose, new Rotation2d());
//AAAaAaaAAaaaaAAa
    }
//AAaaaaaaaaA

//AAaaaAAaaAaaaAa
}
