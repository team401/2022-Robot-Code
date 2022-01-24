package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

    private final WPI_TalonFX rightShooterMotor = new WPI_TalonFX(CANDevices.rightShooterMotorID); //Falcons
    private final WPI_TalonFX leftShooterMotor = new WPI_TalonFX(CANDevices.leftShooterMotorID);  

    private final MotorControllerGroup shooterMotors = new MotorControllerGroup(leftShooterMotor, rightShooterMotor);
        
        
    private final CANSparkMax feederMotor = new CANSparkMax(CANDevices.feederMotorID, MotorType.kBrushless); //775PRO 
    private final CANSparkMax hoodMotor = new CANSparkMax(CANDevices.hoodMotorID, MotorType.kBrushless); //Neo550
    private final CANSparkMax turretMotor = new CANSparkMax(CANDevices.turretMotorID, MotorType.kBrushless); //Neo

    //Encoders
    private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final RelativeEncoder feederEncoder = feederMotor.getEncoder();

    //PID Controllers 
    private final PIDController shooterController = new PIDController(0, 0, 0); //need to use WPILib controller
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

    //TO DO
    //zero the turret
    //set tolerance
    //invert a shooter motor
    public ShooterSubsystem() {

        //sets up turret PID controller
        turretController.setP(turretkP);
        turretController.setI(turretkI);
        turretController.setD(turretkD);

        //sets the turretEncoder position and veloctiy to be based off of radians and radians per sec
        turretEncoder.setPositionConversionFactor(
            2.0 * Math.PI / SuperstructureConstants.turretGearReduction);
        turretEncoder.setVelocityConversionFactor(
            2.0 * Math.PI / (SuperstructureConstants.turretGearReduction * 60.0));
        
    }

    @Override 
    public void periodic() {


    }    

    public void runShooterPercent(double percent) {

        shooterMotors.set(percent);

    }

    public void runFeederPercent(double percent) {

        feederMotor.set(percent);

    }

    public void runHoodPercent() {



    }

    public void runTurretPercent() {


    }

    public double getTurretPositionRadians() {

        //no change is needed in the gear ratio since we set the conversion factor above
        return turretEncoder.getPosition(); 

    }

    //sets the position of the Turret Encoder to 0 (in radians)
    public void resetTurretEncoder() {

        turretEncoder.setPosition(0.0);

    }

    //Uses the Turret PID Controller to go to the desired position
    public void turretSetDesiredClosedState(double desiredPosition) {

        turretController.setReference(
            getTurretPositionRadians(), 
            ControlType.kPosition);

    }

}
