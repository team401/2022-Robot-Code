package frc.robot.subsystems;

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
    //Neo
    private final CANSparkMax turretMotor = new CANSparkMax(CANDevices.turretMotorID, MotorType.kBrushless); 

    //Encoders
    private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final RelativeEncoder feederEncoder = feederMotor.getEncoder();

    //PID Controllers 
    private final ProfiledPIDController shooterController = new ProfiledPIDController(0.005, 0.005, 0, 
        new TrapezoidProfile.Constraints(
            SuperstructureConstants.shooterMaxSpeed * SuperstructureConstants.shooterReduction,
            SuperstructureConstants.shooterMaxAcceleration
        )); //need to use WPILib controller
    private final SparkMaxPIDController hoodController = hoodMotor.getPIDController(); //can use integrated REV
    private final SparkMaxPIDController turretController = turretMotor.getPIDController(); //can use integrated REV

    //**NEED TO CHANGE**
    //PID Values for Turret
    private final double turretkP = 0.0;
    private final double turretkI = 0.0;
    private final double turretkD = 0.0;

    //**NEED TO CHANGE**
    //PID Values for Hood
    private final double hoodkP = 0.0;
    private final double hoodkI = 0.0;
    private final double hoodkD = 0.0;

    private final double tolerance = 0.01; //radians
    private final double shooterTolerance = Units.rotationsPerMinuteToRadiansPerSecond(150);
    private double desiredSpeed;
    private double hoodDesired;

    private Timer timer;
    

    //TO DO
    //zero the turret
    //set tolerance
    public ShooterSubsystem() {

        rightShooterMotor.configFactoryDefault();
        leftShooterMotor.configFactoryDefault();

        rightShooterMotor.setInverted(true);

        //sets up turret PID controller
        turretController.setP(turretkP);
        turretController.setI(turretkI);
        turretController.setD(turretkD);

        //sets up hood PID Controller
        hoodController.setP(hoodkP);
        hoodController.setI(hoodkI);
        hoodController.setD(hoodkD);

        /**
         * sets the turretEncoder position and velocity to be based off of radians and 
         * radians per sec of the structure
         */
        turretEncoder.setPositionConversionFactor(
            2.0 * Math.PI / SuperstructureConstants.turretGearReduction);
        turretEncoder.setVelocityConversionFactor(
            2.0 * Math.PI / (SuperstructureConstants.turretGearReduction * 60.0));

        /**
         * sets the turretEncoder position and velocity to be based off of radians and 
         * radians per sec of the structure
         */        
        hoodEncoder.setPositionConversionFactor(
            2.0 * Math.PI / SuperstructureConstants.hoodGearReduction);
        hoodEncoder.setVelocityConversionFactor(
            2.0 * Math.PI / (SuperstructureConstants.hoodGearReduction * 60.0));

        timer.start();
        timer.reset();

    }

    @Override 
    public void periodic() {

        SmartDashboard.putNumber("desired shooter speed", desiredSpeed);
        SmartDashboard.putNumber("desired hood position", hoodDesired);
        SmartDashboard.putNumber("current hood position", getHoodPositionRadians());

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
     * TURRET METHOD
     */

    //runs turret at given percent
    public void runTurretPercent(double speed) {

        turretMotor.set(speed);
        
    }

    //runs turret based on given voltage for the motor
    public void runTurretVoltage(double volts) {

        turretMotor.setVoltage(volts);

    }

    public double getTurretPositionRadians() {

        //no change is needed in the gear ratio since we set the conversion factor above
        return turretEncoder.getPosition(); 

    }

    //returns velocity in rad per sec
    public double getTurretVelocityCurrent() {

        return turretEncoder.getVelocity();

    }

    //sets value of turret encoder to what is passed in
    public void setTurretEncoder(double desiredPosition){

        turretEncoder.setPosition(desiredPosition);

    }

    //sets the position of the Turret Encoder to 0 (in radians)
    public void resetTurretEncoder() {

        turretEncoder.setPosition(0.0);

    }

    //Uses the Turret PID Controller to go to the desired position
    public void turretSetDesiredClosedState(double desiredPosition) {

        turretController.setReference(
            getTurretPositionRadians(), 
            ControlType.kPosition
        );

    }

    //is it within the bounds?
    public boolean isWithinEdges() {

        return (
            SuperstructureConstants.leftTurretExtrema - getTurretPositionRadians() < tolerance && 
            SuperstructureConstants.rightTurretExtrema - getTurretPositionRadians() > tolerance);

    }

    /**
     * HOOD METHODS
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

        return hoodEncoder.getPosition();

    }

    //returns the velocity of the hood motor in radians per sec
    public double getHoodVelocity() {

        return hoodEncoder.getVelocity();

    }

    //PID controller to set desired position of the hood in radians
    public void hoodSetDesiredClosedState(double desiredPosition) {

        hoodController.setReference(
            desiredPosition,
            ControlType.kPosition
        );

        hoodDesired = desiredPosition;

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

        shooterMotors.set(percent);

    }

    //sets the velocity using the profiled pid
    public void runShooterVelocityProfiledController(double desiredSpeedRadPerSec){

        desiredSpeed = desiredSpeedRadPerSec;
        double powerOut = shooterController.calculate(
             getFlywheelVelocityRadPerSec(),
             desiredSpeedRadPerSec
        );

        shooterMotors.set(powerOut);

    }

    //stops shooter
    public void stopShooter() {

        shooterMotors.set(0);

    }

    public boolean atGoal() {

        return timer.get() >= 0.25;

    }

}
