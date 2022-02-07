// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    //put all of our CAN IDs in here
    public static final class CANDevices {

        //drive motor IDs grouped by swerve module (Falcon 500s)
        public static final int frontLeftRotationMotorID = 1;
        public static final int frontLeftDriveMotorID = 0;

        public static final int frontRightRotationMotorID = 3;
        public static final int frontRightDriveMotorID = 2;

        public static final int backLeftRotationMotorID = 5;
        public static final int backLeftDriveMotorID = 4;

        public static final int backRightRotationMotorID = 7;
        public static final int backRightDriveMotorID = 6;

        //PREP
        //IDs for shooter motors (Falcon 500s)
        public static final int leftShooterMotorID = 8;
        public static final int rightShooterMotorID = 9;

        //PREP
        //IDs for each of the climbing motors (775 Pros + TalonSRX)
        /*public static final int firstStageLeftMotorID = 10;
        public static final int firstStageRightMotorID = 11;
        public static final int secondStageLeftMotorID = 12;
        public static final int secondStageRightMotorID = 13;*/

        //CANcoder IDs for the Swerve Modules
        public static final int frontLeftRotationEncoderID = 10;
        public static final int frontRightRotationEncoderID = 11;
        public static final int backLeftRotationEncoderID = 12;
        public static final int backRightRotationEncoderID = 13;

        //PREP
        //IDs for turret 
        public static final int turretMotorID = 18; //Neo
        public static final int hoodMotorID = 19; //Neo 550
        public static final int feederMotorID = 21; //775 Pro

        //PREP
        //IDs for Intake
        public static final int intakeMotorID = 22; //775 Pro

        //ID for Pigeon
        public static final int imuID = 20;

    }

    //sensors that plug into DIO
    public static final class DIOChannels {

        //For the encoders on each of the swinging arms
        public static final int leftSwingEncoder = 0;
        public static final int rightSwingEncoder = 1;

    }

    //put all of the port IDs in here
    public static final class InputDevices {

        //Joystick ports (will be top 2 on the drive station, make sure the order is right!)
        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

        //gamepad port (3rd on the drive station list)
        public static final int gamepadPort = 2;

    }

    //put any values we access for the drivetrain and/or teleop in here
    public static final class DriveConstants {

        //**NEED TO CHANGE**
        //PID values for drive motor in the Swerve Module class
        public static final double drivekP = 0.05;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;
       
        //**NEED TO CHANGE**
        //PID values for rotation motor in the Swerve Module class
        //Currently tuned down for testing using set voltage control?
        public static final double rotationkP = 0.5;
        public static final double rotationkI = 0.0;
        public static final double rotationkD = 0.0;

        /*
        set factor by dividing by number of units in a rotation of the Falcon500 motor(2048) 
        and multiplying by number of radians in a rotation
        */
        public static final double FalconSensorConversionFactor = 2 * Math.PI / 2048.0;

        //wheel diameter in inches (measured accurately)
        public static final double wheelDiameterMeters = Units.inchesToMeters(3.9028);

        //gear reduction for drive motor
        public static final double driveWheelGearReduction = 6.75;

        //overal conversion factor to get from the drive sensor 
        //(multiply to get radians, then divide for the wheel reduction)
        public static final double driveSensorConversionFactor = 
            FalconSensorConversionFactor / driveWheelGearReduction;

        //**NEED TO CHANGE**
        //static offset values based on how the swerve modules were installed (gotten manually)
        public static final double frontLeftAngleOffset = Units.degreesToRadians(0);
        public static final double frontRightAngleOffset = Units.degreesToRadians(330.38);
        public static final double backLeftAngleOffset = Units.degreesToRadians(0);
        public static final double backRightAngleOffset = Units.degreesToRadians(245.65);

        //**NEED TO CHANGE**
        //constants based on the distance between the center of two wheels
        public static final double trackWidth = Units.inchesToMeters(19.75);
        public static final double wheelBase = Units.inchesToMeters(19.75);

        //**NEED TO CHANGE**
        //Sets max speed for the drive motor, and max speed and acceleration for the rotation motor
        public static final double maxDriveSpeed = 6380.0 / 60.0 
            / SdsModuleConfigurations.MK4_L2.getDriveReduction() * 
            wheelDiameterMeters * Math.PI; //meters per a second
        public static final double maxRotationSpeed = 1.0; //radians per second
        public static final double maxRotationAcceleration = 1.0; //radians per second per second
        public static final double teleopTurnRateDegPerSec = 360.0; //degrees per second
        
        /*
        creating the kinematics object, which will take in the movements we want in terms of dx, dy, and
        rotation and will convert them into states (speed and angle) for each of our swerve modules
        */
        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //values for front left (+, +)
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //values for front right (+, -)
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //values for back left (-, +)
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //values for back right (-, -)
        );

        //**NEED TO CHANGE**
        //values for coefficient of static friction and for velocity (will calculate by characterizing)
        private static final double kS = 0.6271;
        private static final double kV = 0.7470;


        //creates Feed Forward calculator from the characterization values for our drive motors
        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(kS, kV);
    }

    //holds our values for our overall manipulator subsystems (intake, index, shooter)
    public static final class SuperstructureConstants {

        //**NEED TO CHANGE**
        public static final double intakingPower = 0.5;

        //Turret Angle Offsets
        public static final double turretEdge = Units.degreesToRadians(132.5); //what we designate as the edge of the turrret
        public static final double leftTurretExtrema = Units.degreesToRadians(135);  //angle offset from front center of robot, if intake is front
        public static final double rightTurretExtrema = Units.degreesToRadians(-135); //negative in order to match odometry and unit circle

        //**NEED TO CHANGE**
        //Gear reduction/increase in resolution from the gearing on the turret Neo
        public static final double turretGearReduction = 900 * 70; 

        //Gear reduction/increase in resoltuion from the gearing on the hood Neo 550 and Versa Planetarys
        public static final double hoodGearReduction = 80 * 5;
    }

    //put any exclusive auto constants in here
    public static final class AutoConstants{

        //**NEED TO CHANGE**
        //values for our max speed/accleration in auto
        public static final double maxVelocityMetersPerSec = 2.0;
        public static final double maxAccelerationMetersPerSecondPerSecond = 1.0;

    }

}
