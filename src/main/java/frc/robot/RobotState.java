//aaAAAAAAaA
package frc.robot;
//aaaaAAAAaaa

//aAaaAaaAAAA
import edu.wpi.first.math.geometry.*;
//aaaAaaAaa

//AaAaAaa
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//aaAAaaAaAaAA
import edu.wpi.first.wpilibj.DriverStation;
//aAaAaAaAAa
import edu.wpi.first.wpilibj.Timer;
//AaaAaAaaaaAaAaA
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//aaAAA
import frc.robot.util.GeomUtil;
//AaAAAaAaAaaaaAAa
import frc.robot.util.PoseHistory;
//aaaaaAAAAaAAA

//aaAaAAAAAAAaAa
public class RobotState {
//AAaaAAA

//aaAAa
    private static RobotState instance;
//AAaAAaaAa

//AAaAaAaaAaaAaaAaAa
    //If there is no RobotState instance already, make one
//aaaaAAaaAaaAaaAaa
    public static RobotState getInstance() {
//aAaaAaAaAAAaaAaAa
        if (instance == null) {
//aaAaA
            instance = new RobotState();
//aaAaaaaaaAa
        }
//aAaAa
        return instance;
//aaaAAAAaAAAAAAaaaa
    }
//AaAAAAAAaAA

//aaaaa
    private Pose2d fieldToBoot = new Pose2d();
//aaAaaaAAAAa
    private final PoseHistory bootToVehicle = new PoseHistory(100);
//AaAAaAaA
    private final PoseHistory turretFixedToTurret = new PoseHistory(100);
//AaaAAaaaaAAa
    private ChassisSpeeds vehicleVelocity = new ChassisSpeeds();
//AAaAaAAaaaAAaAAa
    private double turretVelocityRadPerSec = 0.0;
//aaAaAaAaaaaAaAa

//AAaaaAaAa
    private Translation2d latestMeasuredFieldToTarget = Constants.FieldConstants.hubCenter;
//AaaaaAAAAAAAaAaaAa

//AaaaAaAaaAAAaAAaA
    private boolean lookAhead = false;
//aaAAaAaAAAa

//aaaAAaaaAAAAaaa
    // 0 = none, 1 = red, 2 = blue
//aaaaaAaaAAAaaaaaaa
    private int currentBall = 0;
//aAaAAaaAAAaaAAA

//AAaaAaAAaaa
    private RobotState() {
//aAAAaaaaA
        bootToVehicle.insert(0.0, new Pose2d());
//aaAAaAAaaaaAaaaAAA
        turretFixedToTurret.insert(0.0, new Pose2d());
//AAaaAaaAAaAAAaaAA
    }
//aaAaaAAAAaaAAAaAaaa

//AAAAaaaaA
    private static double[] poseToDoubleArray(Pose2d pose) {
//aAAAaAAaaAAAaAaAa
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
//aAaAaa
    }
//aAAaA

//aAAaAAaaAaA
    /**
//AaAaaaaa
     * Sets the robot pose to the given pose
//AaaAaaAaAaAAAaAAa
     * 
//aaAaaAaAAA
     * @param fieldToVehicle The robot pose to set
//aAAaAaaaAaAa
     */
//aAaAAaaA
    public void forceRobotPose(Pose2d fieldToVehicle) {
//aAaaAaAAAaaAaAAaAa
        Pose2d bootToVehicle = this.bootToVehicle.getLatest().orElseThrow().getPose();
//aAaAaAaa
        Pose2d vehicleToBoot = GeomUtil.poseInverse(bootToVehicle);
//AaaaaAAAaAaAAAAaA
        fieldToBoot = fieldToVehicle.transformBy(GeomUtil.poseToTransform(vehicleToBoot));
//aAAaaA
    }
//aAaaAAaaAaaaAA

//aAAaaaaaaaAaAAAaA
    public void recordOdometryObservations(Pose2d bootToVehicle, ChassisSpeeds velocity) {
//aaAaAaAaaaAaAa
        this.bootToVehicle.insert(Timer.getFPGATimestamp(), bootToVehicle);
//aAAAaaAaaaAAaaAAAAA
        vehicleVelocity = velocity;
//AaAAAaaaAaaA
    }
//aaAAaaAaAaA

//aAaAAAaAAa
    public void recordTurretObservations(Rotation2d turretFixedToTurretRotation, double turretVelocityRadPerSec) {
//AAaAAAaa
        this.turretVelocityRadPerSec = turretVelocityRadPerSec;
//AAaaaaAaaaaaAA
        turretFixedToTurret.insert(Timer.getFPGATimestamp(), GeomUtil.poseFromRotation(turretFixedToTurretRotation));
//AAAAaAaaAaAaAAAa
    }
//aaaAaaaaAAaaAAaaaaA

//aAaAAAaAaaaa
    public void recordVisionObservations(double captureTimestamp, Translation2d cameraToTarget) {
//AAAaAAaaaaAaaaaa
        latestMeasuredFieldToTarget = getFieldToTurret(captureTimestamp)
//AaaAAAaaaaaaAaAA
                .transformBy(GeomUtil.poseToTransform(Constants.VisionConstants.turretToCamera))
//aaAaaaaAaaAaaaAAa
                .transformBy(GeomUtil.transformFromTranslation(cameraToTarget)).getTranslation();
//aAAaaaAAAaa
    }
//aaaAaaAAaaAaaAAaa

//AaaAaaAAaAAAaa
    public Pose2d getFieldToVehicle(double timestamp) {
//AaAAa
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.get(timestamp).orElseThrow());
//AAAaaAaAAaAAAAAA
        return fieldToBoot.transformBy(bootToVehicle);
//aAAAAaAAAAaAaaAAaa
    }
//aAAAAAaaaAaaAaAAAAA

//aaaaAaa
    public Pose2d getLatestFieldToVehicle() {
//AAaaaaaaaAaAaa
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.getLatest().orElseThrow().getPose());
//AaAaAaAaaa
        return fieldToBoot.transformBy(bootToVehicle);
//AAAaAaAAaaaA
    }
//AaAAaAAAAaaAaaaaAA

//AaaaAAaAaAAaaaAaAaa
    public Pose2d getPredictedFieldToVehicle(double lookaheadTime, double angularLookaheadTime) {
//AAAaaAaa
        return getLatestFieldToVehicle().exp(
//AaaaaAa
                new Twist2d(vehicleVelocity.vxMetersPerSecond * lookaheadTime,
//aAAAAaAAAAa
                        vehicleVelocity.vyMetersPerSecond * lookaheadTime,
//AAaAaa
                        vehicleVelocity.omegaRadiansPerSecond * angularLookaheadTime));
//aAaaaAaAAaaAAa
    }
//aAaaAaaAaA

//AAaaAAaAaaAaaa
    public ChassisSpeeds getVehicleVelocity() {
//aaaaAaAa
        return vehicleVelocity;
//AaaaAAAaaAAaaaaa
    }
//AAAaAA

//AAaAaaAaaa
    public Pose2d getFieldToTurret(double timestamp) {
//AaAaAaaAAAaAaAAAaAA
        return getFieldToVehicle(timestamp).transformBy(GeomUtil.poseToTransform(Constants.TurretConstants.vehicleToTurretFixed)).transformBy(GeomUtil.poseToTransform(turretFixedToTurret.get(timestamp).orElseThrow()));
//AaaAAaaAAAaAaAaA
    }
//AaaAaaaAaAaAaaaaA

//aAAAaaA
    public Pose2d getLatestFieldToTurret() {
//aAAAaAaAaAaaAAAaaa
        return getLatestFieldToVehicle().transformBy(GeomUtil.poseToTransform(Constants.TurretConstants.vehicleToTurretFixed)).transformBy(GeomUtil.poseToTransform(turretFixedToTurret.getLatest().orElseThrow().getPose()));
//aaaAaaaaaaAaA
    }
//AaaAAAAAA

//AAAAaaaAaAAAaaaA
    public AimingParameters getAimingParameters() {
//aAaAA
        Pose2d fieldToPredictedVehicle = null;
//aAaaAaAaaAaaAA
        if (lookAhead)
//aaAAaaaaAa
            fieldToPredictedVehicle = getPredictedFieldToVehicle(Constants.VisionConstants.targetingLookaheadS.get(), Constants.VisionConstants.targetingAngularLookaheadS.get());
//aAAaaAAa
        else
//AAaaAaaaAaa
            fieldToPredictedVehicle = getPredictedFieldToVehicle(0, 0);
//aaAaaaAaaAAaaA
        //Pose2d fieldToPredictedVehicle = getPredictedFieldToVehicle(Constants.VisionConstants.targetingLookaheadS.get(), Constants.VisionConstants.targetingAngularLookaheadS.get());
//AaAAAaaaaaAAa
        Pose2d fieldToPredictedTurretFixed = fieldToPredictedVehicle
//aaAaaaAAAaAaAaA
            .transformBy(GeomUtil.poseToTransform(Constants.TurretConstants.vehicleToTurretFixed));
//aAAAaA
            
//aAaAaAaaaAAAa
        Translation2d turretFixedToTargetTranslation = GeomUtil.poseInverse(fieldToPredictedTurretFixed)
//AaaAaaAaA
            .transformBy(GeomUtil.transformFromTranslation(latestMeasuredFieldToTarget)).getTranslation();
//aaAaAAAAAAAaaaAaAaa

//AaAAaAaAaaaaaaAAaA
        Translation2d vehicleToTargetTranslation = GeomUtil.poseInverse(fieldToPredictedVehicle)
//aaaaAAaAAaaaAaAa
            .transformBy(GeomUtil.transformFromTranslation(latestMeasuredFieldToTarget)).getTranslation();
//aaaaaA

//AaaaaaaaAaaA
        Rotation2d vehicleToGoalDirection = GeomUtil.direction(vehicleToTargetTranslation);
//AaaAAaaAAaAAAA

//aaaaaAaaAAAaAAaAAa
        Rotation2d turretDirection = GeomUtil.direction(turretFixedToTargetTranslation);
//aaAAAaAa
        double targetDistance = turretFixedToTargetTranslation.getNorm();
//AAAaAaaAAAaAaaaAaAa

//AaaaaAaaAaAAaa
        double feedVelocity = vehicleVelocity.vxMetersPerSecond * vehicleToGoalDirection.getSin() / targetDistance - vehicleVelocity.vyMetersPerSecond * vehicleToGoalDirection.getCos() / targetDistance - vehicleVelocity.omegaRadiansPerSecond;
//AaAaaaaaAAa

//AAaAAaaaaAaaaAAAA
        SmartDashboard.putBoolean("LookAhead", lookAhead);
//aAAaaaAAaAa

//aAaAa
        return new AimingParameters(turretDirection, targetDistance, feedVelocity);
//AAaAaAAaAA
    }
//AaAAaAAAaAaAAAA

//Aaaaa
    public Rotation2d getVehicleToGoal() {
//aaAAA

//AaaAAAaaAaaAaAaa
        Pose2d fieldToPredictedVehicle = getPredictedFieldToVehicle(Constants.VisionConstants.targetingLookaheadS.get(), Constants.VisionConstants.targetingAngularLookaheadS.get());
//AAAAAAaAa

//aaaAaaAaA
        Translation2d vehicleToTargetTranslation = GeomUtil.poseInverse(fieldToPredictedVehicle)
//aAaAaAAAaaaa
            .transformBy(GeomUtil.transformFromTranslation(latestMeasuredFieldToTarget)).getTranslation();
//aaAaaAaAaaaaA

//aaAaAAAAAAAAaAA
        return GeomUtil.direction(vehicleToTargetTranslation);
//aaAAaAAaaAaAAAaaA

//aaAAaaaAAAAAAAAaaaA
    }
//aAaAAAAaA

//AAAaAAaA
    public void setLookAhead(boolean b) {
//aAaAaaaAaaAaAaaAA
        lookAhead = b;
//aAAAAAAAaa
    }
//AAAaaaaaAaAaaAAAaA

//aaAAaAAA
    public void setCurrentBall(int ball) {
//aaaaAaaAAaAaaaAaaa
        currentBall = ball;
//aAAaAAAaaAa
    }
//aaAaAaaaaaAa

//AAAaaaaaAAA
    public boolean hasCorrectBall() {
//aaaAAAa
        return (currentBall == 0) ||
//AAaaAaAaAa
               (currentBall == 1 && DriverStation.getAlliance() == DriverStation.Alliance.Red) ||
//AaAaaAAAAAAAAaaa
               (currentBall == 2 && DriverStation.getAlliance() == DriverStation.Alliance.Blue);
//AAAAAaaaAA
    }
//aaaaAaAAaaaaAAA

//AAaAAA
    public final class AimingParameters {
//aAaAA
        
//aaAAAaAAaAa
        //Angle of turret on robot
//aaaaAaaaAAAaaaAA
        private final Rotation2d turretAngle;
//aaAaaaAaaAaaaAA
        //Distance in meters from target
//AaAaAaAaAaAaAa
        private final double distanceM;
//AaaAaAaaAAAAAA
        private final double velocityRadiansPerSec;
//aaAAAaAAaAaA

//aAaAaAa
        public AimingParameters(Rotation2d turretAngle, double distanceM, double velocity) {
//aAAaaAAAaaAaAaaa
            this.turretAngle = turretAngle;
//aaaAaaAaaaAAa
            this.distanceM = distanceM;
//AaAAaAAaAA
            this.velocityRadiansPerSec = velocity;
//aAAAAAAaaaaAaaaAaa
        }
//aaaaaaaaaaaaaaAAaAa

//AaAAAaaAAaaaAA
        public Rotation2d getTurretAngle() {
//AAaaaaaaaA
            return turretAngle;
//AaaAAAAAaAA
        }
//aaaAAaAAAaAAaA
        
//aAaaaaaaAAAaAaaaAa
        public double getDistanceM() {
//AaAaaaaaAaaa
            return distanceM;            
//aAAaaAAAaAAaa
        }
//aAAAaAaaAAaaAAaAA

//aAAaAAAaa
        public double getVelocityRadPerSec() {
//aAaaAA
            return velocityRadiansPerSec;            
//AaAaAaaAAAaAaa
        }
//aAaAaAA

//aaAAaaAAAaA

//AAAAAaAaAaaaaaAAAa
    }
//aAaaA

//AAAaaAaaaAAA
}