package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;

public class ShooterSubsystem extends SubsystemBase {

        private final WPI_TalonFX rightShooterMotor = new WPI_TalonFX(CANDevices.rightShooterMotorID); //Falcons
        private final WPI_TalonFX leftShooterMotor = new WPI_TalonFX(CANDevices.leftShooterMotorID);  

        private final MotorControllerGroup shooterMotors = new MotorControllerGroup(leftShooterMotor, rightShooterMotor);
        
        
        private final CANSparkMax feederMotor = new CANSparkMax(CANDevices.feederMotorID, MotorType.kBrushless); //775PRO 
        private final CANSparkMax hoodMotor = new CANSparkMax(CANDevices.hoodMotorID, MotorType.kBrushless); //Neo550
        private final CANSparkMax turretMotor = new CANSparkMax(CANDevices.turretMotorID, MotorType.kBrushless); //Neo

        //Encoders
        private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
        private final RelativeEncoder hoodEncoder = turretMotor.getEncoder();
        private final RelativeEncoder feederEncoder = turretMotor.getEncoder();

          //PID 
          private final PIDController shooterController = new PIDController(0, 0, 0);
          private final PIDController hoodController = new PIDController(0, 0, 0);
          private final PIDController turretController = new PIDController(0, 0, 0);

    public ShooterSubsystem() {

        //zero the turret
        //set tolerance
        //invert a shooter motor
        
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
        return turretEncoder.getPosition() / (10 * 4 * 10 * 140) * 2 * Math.PI; //fix gear ratio

    }

}
