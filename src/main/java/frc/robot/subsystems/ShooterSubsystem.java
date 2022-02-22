package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

/** Notes:
 * Feeder is only going to be run at a certain percent or backwards (is a Neo 550)
 * Turret is going to be controlled with position with an integrated PID controller (NEO)
 * Hood is going to be controlled with position with WPILib PID controller (Neo 550)
 * Shooter motors are going to be made into a controller group (2 Falcons, WPILib PID contoller for velocity)
 */

public class ShooterSubsystem extends SubsystemBase {

    //Falcon 500s
    private final WPI_TalonFX rightShooterMotor = new WPI_TalonFX(CANDevices.rightShooterMotorID); //Falcons
    private final WPI_TalonFX leftShooterMotor = new WPI_TalonFX(CANDevices.leftShooterMotorID);  

    private final MotorControllerGroup shooterMotors = new MotorControllerGroup(leftShooterMotor, rightShooterMotor);
    
    //775 Pro  
    private final CANSparkMax feederMotor = new CANSparkMax(CANDevices.feederMotorID, MotorType.kBrushless); 
    //Neo550
    private final CANSparkMax hoodMotor = new CANSparkMax(CANDevices.hoodMotorID, MotorType.kBrushless); 

    //Encoders
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final RelativeEncoder feederEncoder = feederMotor.getEncoder();

    //PID Controllers 
    private final ProfiledPIDController shooterController = new ProfiledPIDController(0.005, 0.005, 0, 
        new TrapezoidProfile.Constraints(
            SuperstructureConstants.shooterMaxSpeed * SuperstructureConstants.shooterReduction,
            SuperstructureConstants.shooterMaxAcceleration
        )); //need to use WPILib controller

    private final SparkMaxPIDController hoodController = hoodMotor.getPIDController(); //can use integrated REV

    //**NEED TO CHANGE**
    //PID Values for Hood
    private double hoodkP = 0.1;
    private double hoodkI = 0.0;
    private double hoodkD = 0.0;

    private final double shooterTolerance = Units.rotationsPerMinuteToRadiansPerSecond(150);
    private double desiredSpeed;
    private double hoodDesired;

    private Timer timer = new Timer();
    
    //TODO
    //zero the turret
    //set tolerance
    public ShooterSubsystem() {

        SmartDashboard.putNumber("hoodkP", hoodkP);
        SmartDashboard.putNumber("hoodkD", hoodkD);

        rightShooterMotor.configFactoryDefault();
        leftShooterMotor.configFactoryDefault();

        rightShooterMotor.configNeutralDeadband(0.001);
        leftShooterMotor.configNeutralDeadband(0.001);

        rightShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        leftShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        hoodMotor.restoreFactoryDefaults();

        // Terminal, Constant, Timeout
        leftShooterMotor.config_kF(0, 1023.0/21650, 10);
        leftShooterMotor.config_kP(0, 10, 10);
        leftShooterMotor.config_kP(0, 0, 10);
        leftShooterMotor.config_kP(0, 0, 10);

        rightShooterMotor.setInverted(true);
        hoodMotor.setInverted(true);

        rightShooterMotor.follow(leftShooterMotor);

        //Current Limits
        hoodMotor.setSmartCurrentLimit(20);

        //Soft Limits
        hoodMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        hoodMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        

        //sets up hood PID Controller
        hoodController.setP(hoodkP);
        hoodController.setI(hoodkI);
        hoodController.setD(hoodkD);

        hoodController.setSmartMotionMaxVelocity(6, 0);
        hoodController.setSmartMotionMaxAccel(20, 0);

        /**
         * sets the turretEncoder position and velocity to be based off of radians and 
         * radians per sec of the structure
         */        
        /*hoodEncoder.setPositionConversionFactor(
            2.0 * Math.PI / SuperstructureConstants.hoodGearReduction);
        hoodEncoder.setVelocityConversionFactor(
            2.0 * Math.PI / (SuperstructureConstants.hoodGearReduction * 60.0));*/

        timer.start();
        timer.reset();

    }

    @Override 
    public void periodic() {

        SmartDashboard.putNumber("desired shooter speed", desiredSpeed);
        SmartDashboard.putNumber("desired hood position", hoodDesired);
        SmartDashboard.putNumber("current hood position Revolutions", getHoodPositionRevolutions());

        hoodController.setP(SmartDashboard.getNumber("hoodkP", 0));
        hoodController.setD(SmartDashboard.getNumber("hoodkD", 0));

        if (!(Math.abs(desiredSpeed - getFlywheelVelocityRadPerSec()) < shooterTolerance)) timer.reset();

    }    

    /**
     * FEEDER METHODS
     */

    public void runFeederPercent(double percent) {

        feederMotor.set(percent);

    }

    public void reverseFeederPercent(double percent) {

        feederMotor.set(-percent);

    }

    public void stopFeeder() {

        feederMotor.set(0);

    }

    /**
     * HOOD METHODS
     * 
     * Positive Percent = UP
     * Negative Percent = DOWN
     * 
     */

    //sets value of hood encoder to what is passed in 
    public void setHoodEncoder(double desiredPosition){

        hoodEncoder.setPosition(desiredPosition);

    }

    //sets value of hood encoder to 0
    public void resetHoodEncoder() {

        hoodEncoder.setPosition(0.0);

    }

    //returns position of the hood in radians (with 0 being at the bottom)
    public double getHoodPositionRadians() {

        return hoodEncoder.getPosition() * 2.0 * Math.PI;

    }

    public double getHoodPositionRevolutions() {
        
        return hoodEncoder.getPosition();

    }

    //returns the velocity of the hood motor in radians per sec
    public double getHoodVelocity() {

        return hoodEncoder.getVelocity();

    }

    //PID controller to set desired position of the hood in radians
    public void hoodSetDesiredClosedStateRevolutions(double desiredPosition) {

        hoodController.setReference(
            desiredPosition,
            ControlType.kPosition
        );

        hoodDesired = desiredPosition;

    }

    public void runHoodPercent(double percent) {

        hoodMotor.set(percent);

    }

    public void runHood() {

        hoodMotor.set(0.05);

    }

    public void stopHood() {

        hoodMotor.set(0);

    }

    public void setHoodSoftLimits(float forward, float reverse) {

        hoodMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forward);
        hoodMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverse);

    }

    /**
     * SHOOTER METHODS
     */

    //converts raw sensor units per 100 ms to rad / s
    public double getFlywheelVelocityRadPerSec() {

        return leftShooterMotor.getSelectedSensorVelocity() / 2048 * 2 * Math.PI * 10 
            * SuperstructureConstants.turretGearReduction;

    }

    //runs at percent
    public void runShooterPercent(double percent) {

        //shooterMotors.set(percent);
        leftShooterMotor.set(percent);

    }


    //sets the velocity using the profiled pid
    public void runShooterVelocityProfiledController(double desiredSpeedRadPerSec){

        desiredSpeed = desiredSpeedRadPerSec;
        double powerOut = shooterController.calculate(
             getFlywheelVelocityRadPerSec(),
             desiredSpeedRadPerSec
        );

        //shooterMotors.set(powerOut);
        leftShooterMotor.set(powerOut);

    }

    public void runShooterVelocityController(double desiredRPM) {

        double output = desiredRPM * SuperstructureConstants.shooterMotorRPMConversionFactor;
        leftShooterMotor.set(TalonFXControlMode.Velocity, output);
        SmartDashboard.putNumber("Shooter output", output);

    }

    //stops shooter
    public void stopShooter() {

        //shooterMotors.set(0);
        leftShooterMotor.set(0);

    }

    public boolean atGoal() {

        return timer.get() >= 0.25;

    }

    

}
