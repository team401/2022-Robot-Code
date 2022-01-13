// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class CANDevices {

        //drive motor IDs grouped by swerve module
        public static final int frontLeftRotationMotorID = 0;
        public static final int frontLeftDriveMotorID = 1;

        public static final int frontRightRotationMotorID = 2;
        public static final int frontRightDriveMotorID = 3;

        public static final int backLeftRotationMotorID = 4;
        public static final int backLeftDriveMotorID = 5;

        public static final int backRightRotationMotorID = 6;
        public static final int backRightDriveMotorID = 7;

        //CANcoder IDs for the Swerve Modules
        public static final int frontLeftRotationEncoderID = 13;
        public static final int frontRightRotationEncoderID = 14;
        public static final int backLeftRotationEncoderID = 15;
        public static final int backRightRotationEncoderID = 16;

        //ID for Pigeon
        public static final int imuID = 18;

    }

    public static final class DriveConstants {

        //**NEED TO CHANGE**
        //PID values for drive motor in the Swerve Module class
        public static final double drivekP = 0.0;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;
       
        //**NEED TO CHANGE**
        //PID values for rotation motor in the Swerve Module class
        public static final double rotationkP = 0.0;
        public static final double rotationkI = 0.0;
        public static final double rotationkD = 0.0;

        /*
        set factor by dividing by number of units in a rotation of the Falcon500 motor(2048) 
        and multiplying by number of radians in a rotation
        */
        public static final double FalconSensorConversionFactor = 2 * Math.PI / 2048.0;

        //wheel diameter 
        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0);

        //gear reduction for drive motor
        public static final double driveWheelGearReduction = 6.75;

        //overal conversion factor to get from the drive sensor 
        //(multiply to get radians, then divide for the wheel reduction)
        public static final double driveSensorConversionFactor = 
            FalconSensorConversionFactor / driveWheelGearReduction;

        //**NEED TO CHANGE**
        //static offset values based on how the swerve modules were installed (gotten manually)
        public static final double frontLeftAngleOffset = Units.degreesToRadians(0.0);
        public static final double frontRightAngleOffset = Units.degreesToRadians(0.0);
        public static final double backLeftAngleOffset = Units.degreesToRadians(0.0);
        public static final double backRightAngleOffset = Units.degreesToRadians(0.0);

        //**NEED TO CHANGE**
        //constants based on the distance between the center of two wheels
        public static final double trackWidth = Units.inchesToMeters(100.0);
        public static final double wheelBase = Units.inchesToMeters(100.0);

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
        private static final double kS = 0.0;
        private static final double kV = 0.0;


        //creates Feed Forward calculator from the characterization values for our drive motors
        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(kS, kV);
    }

}
