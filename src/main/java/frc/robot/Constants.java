//AAAaAAaaa
// Copyright (c) FIRST and other WPILib contributors.
//aaAaAaAaAAAA
// Open Source Software; you can modify and/or share it under the terms of
//aaaaaaAAaAAA
// the WPILib BSD license file in the root directory of this project.
//aaaAAA

//AaAAAAaaAAAaAaAaAaA
package frc.robot;
//AAAAAaAaaAaA

//AAaAAAAaaa
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//aaaaaaaAAA
import edu.wpi.first.math.geometry.Pose2d;
//aAAAa
import edu.wpi.first.math.geometry.Rotation2d;
//aAAAAAaAaa
import edu.wpi.first.math.geometry.Translation2d;
//aAaaAaa
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//aAAAAAA
import edu.wpi.first.math.util.Units;
//AaaAaaaaAAaA
import frc.robot.util.GeomUtil;
//aAaaAaaA
import frc.robot.util.TunableNumber;
//AaAAa
import frc.robot.util.Interpolation.InterpolatingDouble;
//AAAaaaAAaaAAA
import frc.robot.util.Interpolation.InterpolatingTreeMap;
//AaAaaaAaAAaAAa

//aAaAa
public final class Constants {
//aaAAaaAa
    public static final boolean tuningMode = false;
//aaAAAaaAAAAAA

//aaaAAAaa
    public static final double trackWidth = Units.inchesToMeters(19.75);
//aaaAa
    public static final double wheelBase = Units.inchesToMeters(19.75);
//AAaaaaaAaaAAaAa
    
//AAAaAAaAaAAAAAAA
    public static final String canivoreName = "Canivore";
//AAaaAaaa

//AAAAAaaAAAAaAaAaa
    public static final class CANDevices {
//aAaaaaAAAAaaAAaaaaA
        
//aAAaaAaA
        public static final int frontLeftDriveMotorID = 0;
//AaaaA
        public static final int frontLeftRotationMotorID = 1;
//AaaAaaAAAaaaAaAaAaa

//AAAAaa
        public static final int frontRightDriveMotorID = 2;
//AAaaaAaAaaaaaAaAAA
        public static final int frontRightRotationMotorID = 3;
//AaAAAaaAAAaa

//aAaAAA
        public static final int backLeftDriveMotorID = 4;
//AaAaAa
        public static final int backLeftRotationMotorID = 5;
//aAAAAaAaa

//AAAAaAaa
        public static final int backRightDriveMotorID = 6;
//aaAAaaAAaaAa
        public static final int backRightRotationMotorID = 7;
//aaAAAAaaA

//AaAAAa
        public static final int leftShooterMotorID = 8;
//aaAAaaaAaA
        public static final int rightShooterMotorID = 9;
//aaaaAAaAAA

//aaaAaaAaaaAaa
        public static final int frontLeftRotationEncoderID = 10;
//AAAAaAAaAAAaaa
        public static final int frontRightRotationEncoderID = 11;
//aAAAaAAAAaA
        public static final int backLeftRotationEncoderID = 12;
//AAAAaA
        public static final int backRightRotationEncoderID = 13;
//AaAAAAAAA

//AAaAAaAaaaaaaaa
        public static final int leftRotationMotorID = 14;
//AaaAAaAaaAAAAAaaaA
        public static final int rightRotationMotorID = 15;
//aaaAAaaAAA
        public static final int leftTelescopingMotorID = 16;
//aAAaa
        public static final int rightTelescopingMotorID = 17;
//AAaAaaA

//aAAAAa
        public static final int turretMotorID = 18;
//aAAaAaa
        public static final int hoodMotorID = 19;
//AaaaAAaaaAaAa
        public static final int turretEncoderID = 30;
//aAAAA

//AAaAaAaaAaaAaaaa
        public static final int pigeonIMU = 20;
//AAaAAAaAaAaAAaaa

//aaAaaAaAaaAAAaAAAa
        public static final int leftLidar = 21;
//AAaaAaaaaAAAaAA
        public static final int rightLidar = 22;
//aaaAAaaAa

//aaaAAa
        public static final int intakeMotorID = 22;
//aAaAAaAA
        public static final int conveyorMotorID = 23;
//AaaAAAaA
        public static final int indexMotorID = 24;
//Aaaaaa

//aaAAAAA
    }
//aAAaAAAaAaAAAAAaaa

//aAAaa
    public static final class DIOChannels {
//aAAaAaAaAA

//aaAAaaaaAAaaaa
        public static final int topBannerPort = 1;
//aAaaaAAaaaAAaaaaA
        public static final int bottomBannerPort = 6;
//AaaAaAaAa

//aaaAAaAaaaaA
        public static final int leftRotationArmEncoder = 2;
//aAAaaaAAaAaAaAaaAa
        public static final int rightRotationArmEncoder = 3;
//aAaaAAAaAAAAaAAAAA

//aAaAaaaAAAaaaAaA
        public static final int turretEncoderPulse = 7;
//aAaaAaA
        public static final int turretEncoderA = 8;
//AAAAAAA
        public static final int turretEncoderB = 9;
//AaAAAAaAa

//AaAAA
    }
//AaAAAA
    
//aaAAaaAA
    public static final class DriveConstants {
//AaAAaaAa
        public static final double driveWheelGearReduction = 6.75;
//AaaaaAa
        public static final double rotationWheelGearReduction = 150.0 / 7.0;
//aAAAaaaAAAAAaAAaaA
        public static final double maxSpeedMPerS = Units.feetToMeters(15.0);
//AaAaaaaaAAAaAaAA
        public static final double maxSpeedWhileShootingMPerS = Units.feetToMeters(10.0);
//AAaAaaaaaaAaAaaaaaa
        public static final double maxAngularSpeedRadPerS = 2 * Math.PI;
//aAaaAaAAaAaaaAAaa

//AaaaaAAa
        public static final double wheelRadiusM = Units.inchesToMeters(3.9028) / 2.0;
//aAaaaaaAaa

//aaaAaaaAAaAaAaaaa
        // Measured on 3/9/22 with machinist's square
//aaaAAAaaaaaAaAaaAa
        // TODO verify these with 1x1 and clamping
//aaAaaaaaaaAA
        public static final double frontLeftAngleOffset = 0.8605632220038447;
//AaAaAAaAAAaAaAAAA
        public static final double frontRightAngleOffset = 5.750893973783269;
//aaaaAAAA
        public static final double backLeftAngleOffset = 1.2854759002481673;
//AAAaAaaAaAAAaaaAa
        public static final double backRightAngleOffset = 4.275204455837282;
//aAaAaaAAaaaAAaA

//aAaaaA
        public static final TunableNumber rotationKp = new TunableNumber("Drive/RotationKp");
//aaAAAaAaaAA
        public static final TunableNumber rotationKd = new TunableNumber("Drive/RotationKd");
//AAaaAaA
        public static final TunableNumber driveKp = new TunableNumber("Drive/DriveKp");
//AAaAAaaaAA
        public static final TunableNumber driveKd = new TunableNumber("Drive/DriveKd");
//AaaAAAaAAAAAA

//aaAAaaaaAAaaaaaAAaA
        public static final TunableNumber followTrajectoryXControllerKp = new TunableNumber("Drive/FollowTrajectoryXControllerKp");
//aAAAAaaAaAaa
        public static final TunableNumber followTrajectoryXControllerKd = new TunableNumber("Drive/FollowTrajectoryXControllerKd");
//AaaaAAAaAaAA

//AaaaAaaAaaaAAaAaaa
        public static final TunableNumber followTrajectoryYControllerKp = new TunableNumber("Drive/FollowTrajectoryYControllerKp");
//aaaaAaaAaa
        public static final TunableNumber followTrajectoryYControllerKd = new TunableNumber("Drive/FollowTrajectoryYControllerKd");
//aAAAaAaaaAaAa

//AaaAAAaAAaa
        public static final TunableNumber followTrajectoryThetaControllerKp = new TunableNumber("Drive/FollowTrajectoryThetaControllerKp");
//AAaaaaAAaaAA
        public static final TunableNumber followTrajectoryThetaControllerKd = new TunableNumber("Drive/FollowTrajectoryThetaControllerKd");
//AaAaaAAaAA

//AAaAaaAAaaA
        static {
//AaAAAaA
                followTrajectoryXControllerKp.setDefault(0.25);
//AAaaa
                followTrajectoryXControllerKd.setDefault(0);
//aAaAaAa

//aAAAAaaaAaa
                followTrajectoryYControllerKp.setDefault(0.25);
//aAaAAAA
                followTrajectoryYControllerKd.setDefault(0);
//AaaaAAaaAaAaaaAAaaA

//AAaAAaaaAAA
                followTrajectoryThetaControllerKp.setDefault(3.0);
//AaAAaAAa
                followTrajectoryThetaControllerKd.setDefault(0);
//AAaaaaaAaaaaAAaAa

//aAaaAaAaAAa
        }
//AaaAaaaaAaAaAaaA

//aAAAAaa
        public static final double driveJoystickDeadbandPercent = 0.075;
//aaaAAAAAaAAAAAA
        public static final double driveMaxJerk = 200.0;
//aAAaaAAAAAAAaa
        
//aaAaaaaa

//AAaaAaaaAAaAAaAa
        static {
//aAaaAAaAAaAaAaAAaA
            // Tuned on 9/19/22
//aaAaaaaaaaaAa
            rotationKp.setDefault(4.7);
//aAaaaaaAAaaaAAaaaaa
            rotationKd.setDefault(0.1);
//AaAAaaAAA

//aaAAAaaAAAAa
            driveKp.setDefault(0.2);
//AAaaAAa
            driveKd.setDefault(2.0);
//AAAaAa
        }
//aAAAaAAaaaaAAAaaAAa

//AAaaAAaAAaAAaAAAAa
        public static final SwerveDriveKinematics kinematics = 
//aaAAaAAAaaaaAa
            new SwerveDriveKinematics(
//aAaaAaaAAAA
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //values for front left (+, +)
//aAAaaAAaAaaAA
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //values for front right (+, -)
//AAaAAaaaAAaa
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //values for back left (-, +)
//AaAaaAAaa
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //values for back right (-, -)
//aAaAaaaAAAaAaaAaa
            );
//AAAAAAaAaaAaaAaaaA

//AaAAaaAAAaaaa
        public static final SimpleMotorFeedforward driveModel = new SimpleMotorFeedforward(0.184, 0.1163414634);
//AAAaAAaAa

//aAAAA
        public static final TunableNumber intakeVisionKP = new TunableNumber("intakeVisionKp");
//aaaAaAAaa
        public static final TunableNumber intakeVisionKD = new TunableNumber("intakeVisionKd");
//AaaaAaaAAaaAAA

//aaAAAaA
    }
//aAAaaaaAAaaaAAAAAA

//aAaaAAaAAAAAaaaAaaa
    public static final class ClimberConstants {
//aaaAaAAAAaAAaaaAAAa
        public static final double rotationPositionToleranceRad = 10;
//aAAaaaAAAa

//AAAAaAaAa
        public static final double rotationMax = Units.degreesToRadians(50);
//AAAAaaAAaAa
        public static final double rotationMin = -0.245;
//AaAAaAAAAaA

//aAAAAaAAaaaAAAAaAa
        public static final double stowPositionRad = rotationMin;
//aAaaAAaa
        public static final double climbGrabPositionRad = Units.degreesToRadians(2);
//aaAaaaaAaaAa
        public static final double intakePositionRad = Units.degreesToRadians(25);
//aaAAAAaAaAAaaa
        public static final double climbSwingPositionRad = Units.degreesToRadians(30);
//aaaaA
        public static final double rotationLatchRad = Units.degreesToRadians(17);
//AAaAaaAaA

//AAaaaAaAaAa
        // measured offsets
//AAaaaaAAAAaaAAA
        public static final double leftRotationOffset = 0.246;
//AaAaaaaa
        public static final double rightRotationOffset = 0.243;
//AAAaaAaAaaaAaaAaaA

//aaAaaAAAaAaa
        public static final double maxHeightMeters = 0.971;
//AAAaAAAaaaAaaa

//aAaaaaaaAaAaAaAa
        // New Telescope Constants
//aAaaAAaa
        public static final double telescopeConversionFactor = 1.01 / 230.0; // rotations to meters
//aAaaaaaAAAaAAaAa

//aAaAaAAaaa
        public static final double telescopeOffsetM = 0.13;
//aaAAAAaaaAAaaa

//AaAaaaaaAA
        public static final double telescopeMaxPositionM = 1.01;
//AaaaaaaaA
        public static final double telescopeDefaultPositionM = 0.08;
//aaAAAaaaaaAaaaaAAa
        public static final double telescopeHomePositionM = 0.05;
//aaaAAaaAAAAaaaaaAAA
        public static final double telescopePullPositionM = 0;
//aAaaA
        public static final double telescopePopAboveRungM = 0.2;
//AAAaA
        public static final double telescopeLatchM = 1.0;
//AAAaaAaaaaAAaAaaa
        public static final double telescopeRotationSafePositionM = 0.47;
//AaaAAAaAaaaAaa
        public static final double telescopeSwingPositionM = 0.55;
//AAaAAaAAAaaaAaaaAa
        
//aaAaAa
        public static final double telescopeGoalToleranceM = 0.05;
//AaAaAaaAAaAAaaaa
        
//aAaAAa
        public static final double telescopeCruiseVelocityM = 0.5;
//AaaAaAAaaaaAaA
        public static final double telescopeAccelerationM = telescopeCruiseVelocityM * 4;
//AaaAaaAaa
        
//aaaAaAaaAaaaaAaAaA
        public static final double climberSequencePauseSeconds = 2;
//aaAAAaA

//aAAaAAaaaaaaAaAA

//aaAAaAaAAA
        //Tunable PD Numbers
//AaaaaAAAaaaAaaAAaA
        public static TunableNumber rotationArmKp = new TunableNumber("RotationArm/Kp");
//AaaAa
        public static TunableNumber rotationArmKd = new TunableNumber("RotationArm/Kd");
//AaAaAAaaaAaAAAaa
        public static double telescopeArmKp = 30.0;
//AaAAAAAAaaaaAaaAaa
        public static double telescopeArmKd = 0.1;
//aAAaAAAAaaaaaAaaa

//aaAaAAAAAaAaa
        public static final double telescopeHomingThresholdMPerS = 0.05;
//AAAaaaAAAaAAAa
        public static final double homingTimeS = 0.2;
//AaaaaAAAAaaAaAAaaA
        public static final double telescopeHomingVolts = -4;
//AAaAaaAa
        public static final double rotationHomingVolts = -4;
//aAAaAa

//aaAaAAaaaAAAAaaAA
        public static final double rotationHomingThresholdRadPerS = 0.1;
//AAaaaAaaaAaaAa

//aAAaAAaAAaaAAaA
        static {
//aAaaaAaaAAAaAAaa
            rotationArmKp.setDefault(25.0);
//aaAaaaAAaaAaaA
            rotationArmKd.setDefault(0.5);
//aAaAAaAaAaA
        }
//AaaaAAAAAaaaaaaAAa

//AaaaAaaAaaAAAA
    }
//aAAAAAAAAAaAaaa

//AAAAAaAAAaaaAAaa
    public static final class ShooterConstants {
//aAaaaAaAAaaAA
        public static final double hoodRackRatio = 5.23 * (458.0 / 30.0);
//aaAAAAAaaA
        public static final double hoodOffsetRad = Math.atan(2.336 / 9.800); // From CAD
//AAAAaaaAaAaA

//AaaaaAaAAAAAaAAaa
        public static final double hoodHomingThresholdRadPerS = Units.degreesToRadians(10);
//AaaaaAaaa
        public static final double hoodHomingTimeS = 0.5;
//aaAAAaAaAaAAaaaAAa
        public static final double hoodHomingVolts = -4;
//aAAAaa

//AAAaAAAaaAaAAAa
        public static final TunableNumber hoodKp = new TunableNumber("Shooter/HoodKp");
//aaaaAaAAaaaa
        public static final TunableNumber hoodKd = new TunableNumber("Shooter/HoodKd");
//AaaaaAaa
        public static final TunableNumber flywheelKp = new TunableNumber("Shooter/FlywheelKp");
//aAaaaaAAAaaAaaA
        public static final TunableNumber flywheelKd = new TunableNumber("Shooter/FlywheelKd");
//aAAaaAaaAAaaaAAAA

//aAAaa
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelLookup = new InterpolatingTreeMap<>();
//AaaaAaAAa
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodLookup = new InterpolatingTreeMap<>();
//aAAaAaaaAAA

//AAAaaaAaaAA
        public static final SimpleMotorFeedforward flywheelModel = new SimpleMotorFeedforward(0.0539, 0.0190538);
//AaAAAaAaA
        
//AaAaaAaaAaAaAaAaaa
        public static final double hoodMinRad = 0.27;
//AAAAaa
        public static final double hoodMaxRad = 0.63;
//AAaaaaaAa
        
//aaAaAAa
        public static final double maxDistanceToTargetIn = Units.metersToInches(4.5);
//AAaaAaaaaaaAaaa

//aAAaAaaaAAaaaAAAaAa
        public static final TunableNumber flywheelDesired = new TunableNumber("Shooter/FlywheelDesired");
//AAaAAAAA
        public static final TunableNumber hoodDesired = new TunableNumber("Shooter/HoodDesired");
//aaaAAaAaAAAaA

//aaAAAaaA
        public static final double intentionalMissRPM = 1000;
//aaAAAaaaaaAAaaaAA

//AAAAAaaaaAaAa

//AAaAAAAAAaa
        static {
//AaAAaaaAaAa
            hoodKp.setDefault(0.7);
//AaaAAaAaA
            hoodKd.setDefault(0);
//aAaAAaAaaaAAA
            flywheelKp.setDefault(0.07);
//aaaAaaaaa
            flywheelKd.setDefault(4);
//AAaaaAAaaaa

//AAAAaaa
            flywheelDesired.setDefault(0);
//AaAAaAaaaa
            hoodDesired.setDefault(hoodMinRad);
//AAAAaaAaAAaAaa

//aAaAaAAAAAaaAAaa
            // Constraints: shooter should not go below 0 RPM (lol) or above ~3000 RPM
//aaaAa
            flywheelLookup.put(new InterpolatingDouble(1.8), new InterpolatingDouble(2050.0));
//AaaAAAaaaaAaaaaa
            flywheelLookup.put(new InterpolatingDouble(2.2), new InterpolatingDouble(2125.0));
//aAaAAaaAaaaaAAaAA
            flywheelLookup.put(new InterpolatingDouble(2.8), new InterpolatingDouble(2125.0));
//aAAaaa
            flywheelLookup.put(new InterpolatingDouble(3.2), new InterpolatingDouble(2250.0));
//Aaaaa
            flywheelLookup.put(new InterpolatingDouble(3.6), new InterpolatingDouble(2400.0));
//aAaAaaaAAAaAa
            
//aaAAaa
            // Constraints: hood should not go below 0.27 or above 0.63
//aAAaaa
            hoodLookup.put(new InterpolatingDouble(1.8), new InterpolatingDouble(0.4));
//AAAAA
            hoodLookup.put(new InterpolatingDouble(2.2), new InterpolatingDouble(0.5));
//aAAaaaaa
            hoodLookup.put(new InterpolatingDouble(2.8), new InterpolatingDouble(0.63));
//AaaaAaaaAAa
            hoodLookup.put(new InterpolatingDouble(3.2), new InterpolatingDouble(0.63));
//AaAAAA
            hoodLookup.put(new InterpolatingDouble(3.6), new InterpolatingDouble(0.63));
//aaaAAA
            
//AAaaaaAAAA
        }
//aaAAAAaaAaAaaAAA

//aaaaAAaAAaAA
    }
//AaAaAAAAAAAaaA

//aaaaAaaaAaaAaaaAa
    public static final class BallConstants {
//AaaAaAaAAAaaAaAAaa
        public static final TunableNumber intakePower = new TunableNumber("Intake/IntakePower");
//AAAAaAAa
        public static final double towerPower = 0.5;
//AAAAaAaAa

//aAaAAAaAaaaAaa
        static {
//AAaaaaAaaAAAAaAAA
                intakePower.setDefault(1.0);
//AaaAAAaA
        }
//AAAaA
    }
//AAAaaaAAAaAAA

//aAAAaAAaAaAa
    public static final class TurretConstants {
//aAAaaaAAaa
        public static final double turretGearRatio = 1.0;
//AAaaaAAAAaaA

//AAAaa
        public static final Pose2d vehicleToTurretFixed = GeomUtil.inchesToMeters(new Pose2d(-5.25, 0.0, Rotation2d.fromDegrees(180)));
//aAAAaaaAaaaAa

//aaAaaa
        public static final double turretEncoderOffsetRad = -1.0631419667728104+0.059825250727540004+0.15;//-0.23344302303378667;
//AAAaaaaaaAaaaa

//AAAAAAAAA
        public static final double turretLimitLower = -Math.PI / 2.0;
//AaAAAA
        public static final double turretLimitUpper = Math.PI / 2.0;
//AaaAaaAAa

//AAaAaAAaaaAaAaAaaaA

//aaAaaAaAAaaaAAAA
        public static final TunableNumber velocityKp = new TunableNumber("Turret/VelKp");
//AaaAAaAaAAaAaA
        public static final TunableNumber velocityKd = new TunableNumber("Turret/VelKd");
//aaaAaaaaaaaA
        public static final TunableNumber positionKp = new TunableNumber("Turret/PosKp");
//AaAaaaaaaaaAAAa
        public static final TunableNumber positionKd = new TunableNumber("Turret/PosKd");
//AaaaaaaAa

//aAaaAAAaaAAaAAA
        static {
//AAaaAAaAAAAaAa
            velocityKp.setDefault(0);
//AaaAAAaAAAaAAa
            velocityKd.setDefault(0);
//aaaaa
            positionKp.setDefault(45);
//AaaAAaaaaaAAAaAaaaa
            positionKd.setDefault(0.2);
//aAaaAa
        }
//AaaAaaAAAAaAaaaa

//aaaaaaaaAAaaAAa
        public static final SimpleMotorFeedforward turretModel = new SimpleMotorFeedforward(0.204, 2.20697674);
//AAaAAaaAaAAA

//AaaaaaAAaaaaaAaaAA
        public static final int setupCycleCount = 20;
//aaaaAaaAAaAaaAaa
    }
//aaaaAaaAaaaAa

//aAAAAaA
    public static final class VisionConstants {
//aAAAaaaaaAaAaaa
        public static final double widthPixels = 960;
//AaaaAAaaAAA
        public static final double heightPixels = 720;
//AAAAAAaAaAaaa
        public static final Rotation2d fovHorizontal = Rotation2d.fromDegrees(59.6);
//aaaAaAaAAAaAa
        public static final Rotation2d fovVertical = Rotation2d.fromDegrees(49.7);
//aAaaAaaAaAAAAaAAaA
        public static final double cameraHeightM = Units.inchesToMeters(26.517);
//AAAaAAa
        public static final Pose2d turretToCamera = GeomUtil.inchesToMeters(new Pose2d(6.461, 0.0, new Rotation2d()));
//AAaAaAAAAaAaAAaAaAa

//AaaaaaaAaaAAa
        public static final TunableNumber targetingLookaheadS = new TunableNumber("Targeting/LookaheadS");
//aAAaAaAAA
        public static final TunableNumber targetingAngularLookaheadS = new TunableNumber("Targeting/AngularLookaheadS");
//aAaaaaAaAAaaAa

//AaAaaaaaaaaa
        public static final TunableNumber floorToCameraAngleDeg = new TunableNumber("Vision/FloorToCameraDeg");
//AaaaAA

//aAaAaAaAAAa
        static {
//AAaAaaAAaAAAaAAaa
            floorToCameraAngleDeg.setDefault(51.0);
//aAaaaAAaaaaAaaaAAa
            targetingLookaheadS.setDefault(0.7);
//AaAaaa
            targetingAngularLookaheadS.setDefault(0.15);
//aAAAaAaaAaaAAAAA
        }
//aaAaaaAAAaaAAAA

//aaaaaAA
        // Vision target
//aaaaaAAaaaaaa
        public static final double visionTargetDiameter =
//aaaaA
                Units.inchesToMeters(4.0 * 12.0 + 5.375);
//AAaaaAA
        public static final double visionTargetHeightLower =
//aaaaAAaAAaaaAaaA
                Units.inchesToMeters(8.0 * 12 + 5.625); // Bottom of tape
//aaaAAaAaaaAaaAAAa
        public static final double visionTargetHeightUpper =
//aaaAaAAAAaAaAAaaAAa
                visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape
//AaAaaaaAaaaAAA

//aAAAaAaAaA

//AAaaAaA
        static {
//aAaaAaAaAAaaaaAaaAa
            floorToCameraAngleDeg.setDefault(45.0);
//AaAaAAaAaaAaaa
        }
//AAaAaAAAaaAaaaAAAA
    }
//AAAAAAaaaaaaAAAAaa

//aAaaA
    public static final class FieldConstants {
//aaaAaaaaaa
        // Field dimensions
//AaAaAaaaAAAAaaA
        public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
//AAAaAAaaaAAAAaaA
        public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
//aaaaaaAAaAaAaaAAaaA
        public static final double hangarLength = Units.inchesToMeters(128.75);
//aaaAaaAaAAaaaa
        public static final double hangarWidth = Units.inchesToMeters(116.0);
//AAAAAaaAAaaaAAAAAa

//AAAAAAaaaAaA
        // Vision target
//AAaAa
        public static final double visionTargetDiameter =
//aAaAAAAAaaAAaAaAaA
                Units.inchesToMeters(4.0 * 12.0 + 5.375);
//AaaAAAAAAAAAA
        public static final double visionTargetHeightLower =
//AaaAAaAaa
                Units.inchesToMeters(8.0 * 12 + 5.625); // Bottom of tape
//aAaAaa
        public static final double visionTargetHeightUpper =
//aaAaAAAaaaAAAaAa
                visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape
//aaaaaA

//aaAaaAaaaa
        // Dimensions of hub and tarmac
//AaAaAAAAAAa
        public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
//AaAaAA
        public static final Translation2d hubCenter =
//AaAAaaAaaAaaaAa
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
//aAAaA
        public static final double tarmacInnerDiameter = Units.inchesToMeters(219.25);
//AAaaaAaaAaAAa
        public static final double tarmacOuterDiameter = Units.inchesToMeters(237.31);
//aaAaAaaAaAaAa
        public static final double tarmacFenderToTip = Units.inchesToMeters(84.75);
//AAaaAAaA
        public static final double tarmacFullSideLength =
//AAAAaAaAAaaAAaA
                tarmacInnerDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
//AAAaaAaAaaAAAaaa
        public static final double tarmacMarkedSideLength =
//aaAAaaaaaa
                Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
//aAAaAAaAaA
        public static final double tarmacMissingSideLength =
//aAaaaAAaaaAaAA
                tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff
//AAAAAaaA
        public static final double hubSquareLength =
//aAaAAAaaaAAa
                tarmacOuterDiameter - (tarmacFenderToTip * 2.0);
//AAAAAAAaAAAaaaaa

//aaaaAaaaaaaAAAa
        // Reference rotations (angle from hub to each reference point and fender side)
//aAaaa
        public static final Rotation2d referenceARotation =
//aaaAaAaaaa
                Rotation2d.fromDegrees(180.0).minus(centerLineAngle)
//AAaaAa
                        .plus(Rotation2d.fromDegrees(360.0 / 16.0));
//aaAAaAaaAaA
        public static final Rotation2d referenceBRotation =
//aaAAAaAAAaA
                referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
//aAAaaaAAAaaaAaa
        public static final Rotation2d referenceCRotation =
//aAaAaaAaaaAAaAaa
                referenceBRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
//aaAaAaaaAAaAAaaaa
        public static final Rotation2d referenceDRotation =
//AAaAAA
                referenceCRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
//AAAaAAAA
        public static final Rotation2d fenderARotation =
//AaaAAAAAAaAAaaaaaA
                referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 16.0));
//aaAAaAaaAaAaaAaAaA
        public static final Rotation2d fenderBRotation =
//aAAaa
                fenderARotation.rotateBy(Rotation2d.fromDegrees(90.0));
//AAAaaaaAAaAAaaAa
        public static final Rotation2d fenderCRotation =
//aAaAa
                fenderBRotation.rotateBy(Rotation2d.fromDegrees(90.0));
//aaaaaaAaaA
        public static final Rotation2d fenderDRotation =
//AaAaa
                fenderCRotation.rotateBy(Rotation2d.fromDegrees(90.0));
//aaAAAaaaaAaaaaAaa

//aaaaAAa
        // Reference points (centered of the sides of the tarmac if they formed a complete octagon, plus
//aaaAaAaAaAaaAA
        // edges of fender)
//AaAaAAAa
        public static final Pose2d referenceA =
//AAaaA
                new Pose2d(hubCenter, referenceARotation).transformBy(
//AaAAAaa
                        GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
//AaAaaAaaAAaAA
        public static final Pose2d referenceB =
//aAaaAAA
                new Pose2d(hubCenter, referenceBRotation).transformBy(
//AAaaAaaaaAAAAAAa
                        GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
//aaAaaAAaAAAaaa
        public static final Pose2d referenceC =
//AAaaaA
                new Pose2d(hubCenter, referenceCRotation).transformBy(
//aAaAaAAaaaA
                        GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
//AAAaa
        public static final Pose2d referenceD =
//aAaaaAAAaAAAAAAAa
                new Pose2d(hubCenter, referenceDRotation).transformBy(
//AAAAAAa
                        GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
//AAaaAAaAAaAaaAa
        public static final Pose2d fenderA =
//AAAAA
                new Pose2d(hubCenter, fenderARotation).transformBy(
//aAAaAAAAaaaA
                        GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
//AAaAA
        public static final Pose2d fenderB =
//AAaaAAaa
                new Pose2d(hubCenter, fenderBRotation).transformBy(
//AAAaaaaaAAaAaAA
                        GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
//aAaaAaAaAaAAa
        public static final Pose2d fenderC =
//aAaaAaaaaaaAAA
                new Pose2d(hubCenter, fenderCRotation).transformBy(
//aaaaaAaaAAAAaAaAaaA
                        GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
//aAAAaAaAAaAaa
        public static final Pose2d fenderD =
//AAAaaAaaaaaaaaAaaa
                new Pose2d(hubCenter, fenderDRotation).transformBy(
//aaaAAaaaaaAaAaaaAaa
                        GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
//aAaaAaaaaaAaAAAA

//aaaAa
        // Cargo points
//AaAAaaaAaa
        public static final double cornerToCargoY = Units.inchesToMeters(15.56);
//AAaaAAaaaA
        public static final double referenceToCargoY =
//aaaaaaAAaaAaA
                (tarmacFullSideLength / 2.0) - cornerToCargoY;
//aAAaAAaAaaAA
        public static final double referenceToCargoX = Units.inchesToMeters(40.44);
//aAAaAaaaAaAA
        public static final Pose2d cargoA = referenceA.transformBy(
//AAaAAAAaaAAaAaAa
                GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
//aaAaaaAAAAAaA
        public static final Pose2d cargoB = referenceA.transformBy(
//aaaAaAAaaAa
                GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
//AaaaAA
        public static final Pose2d cargoC = referenceB.transformBy(
//aAAaAaAaAAaAaAaAAAA
                GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
//AaAaAAaAAAaaaA
        public static final Pose2d cargoD = referenceC.transformBy(
//AAAAAAaa
                GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
//AAaaAaaAaAaaaAAAAaa
        public static final Pose2d cargoE = referenceD.transformBy(
//AAaAaA
                GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
//AaAaA
        public static final Pose2d cargoF = referenceD.transformBy(
//AAaaAAaAaaaaAAaaAaA
                GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
//AAAaaaaaaAaAaAAaA

//aaAaaAaaaa
        // Terminal cargo point
//AaAAaAaaAAAAA
        public static final Rotation2d terminalOuterRotation =
//AAaaAAaaa
                Rotation2d.fromDegrees(133.75);
//AAaaAAAaaAaaAaa
        public static final double terminalLength =
//AaAaaaAAaAAAAaaaa
                Units.inchesToMeters(324.0 - 256.42);
//AAAAAAaaAAaaaAaa
        public static final double terminalWidth = Math.tan(
//AaaaaaaAaAAaaaAA
                Rotation2d.fromDegrees(180.0).minus(terminalOuterRotation).getRadians())
//aaAAAAAA
                * terminalLength;
//AaAAaAAAAAaa
        public static final Pose2d terminalCenter =
//AaaaAAaAAaAAa
                new Pose2d(new Translation2d(terminalLength / 2.0, terminalWidth / 2.0),
//AAaAAAAAaAA
                        terminalOuterRotation.minus(Rotation2d.fromDegrees(90.0)));
//AaAAa
        public static final double terminalCargoOffset = Units.inchesToMeters(10.43);
//aaaaaaaAaaAAaAAa
        public static final Pose2d cargoG = terminalCenter
//AAaaAaa
                .transformBy(GeomUtil.transformFromTranslation(terminalCargoOffset, 0.0));
//AAAAAAAaaAaAaaaaaAA
    }
//aAAAaAaAAaaaAAAaa

//aaAAaaaAaAaAAaa
    public static final class AutoConstants {
//AAAAaaAAaaaaAAaA
        public static final double kMaxVelocityMetersPerSecond = 4.0;
//Aaaaa
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
//aAaAAaaaAaaaaAaa
    }
//AAaAAaaaaAAAaAA

//aaAAAAaaAAA
    public static final class LEDConstants {
//aAaAAaaaaaAaaaaa
        public static final int stipLength = 60;
//AaAaAAAA
    }
//AAAaAAa

//AaAaaA
}
//aAaAAaAaaaaaaaAaAA

