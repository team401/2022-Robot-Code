package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

public class TurretSubsystem extends SubsystemBase {
    
    //Neo
    private final CANSparkMax turretMotor = new CANSparkMax(CANDevices.turretMotorID, MotorType.kBrushless); 

    //Encoders
    private final RelativeEncoder turretEncoder = turretMotor.getEncoder();

    //can use integrated REV PID controller
    private final SparkMaxPIDController turretController = turretMotor.getPIDController(); 

    //**NEED TO CHANGE**
    //PID Values for Turret
    private final double turretkP = 0.0;
    private final double turretkI = 0.0;
    private final double turretkD = 0.0;

    private final double tolerance = 0.01; //radians
    
    public TurretSubsystem() {

        //sets up turret PID controller
        turretController.setP(turretkP);
        turretController.setI(turretkI);
        turretController.setD(turretkD);

        /**
         * sets the turretEncoder position and velocity to be based off of radians and 
         * radians per sec of the structure
         */
        turretEncoder.setPositionConversionFactor(
            2.0 * Math.PI / SuperstructureConstants.turretGearReduction);
        turretEncoder.setVelocityConversionFactor(
            2.0 * Math.PI / (SuperstructureConstants.turretGearReduction * 60.0));


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
}
