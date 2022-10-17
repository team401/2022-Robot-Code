package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;

public class DriveAngle {

    private final Pigeon2 pigeon = new Pigeon2(CANDevices.pigeonIMU, Constants.canivoreName);

    private double degOffset = 0;

    public double headingRad;

    public void updateHeading() {
        headingRad = Units.degreesToRadians(pigeon.getYaw() - degOffset);
    }

    public void resetHeading() {
        degOffset = pigeon.getYaw();
    }  
    
}