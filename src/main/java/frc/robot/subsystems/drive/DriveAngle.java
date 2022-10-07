//AAAAa
package frc.robot.subsystems.drive;
//aAaaaaAaaaA

//AAaAA
import com.ctre.phoenix.sensors.Pigeon2;
//aaAAaAaaAAAaaAaAa

//aaaAAAAAaAAAaAAAA
import edu.wpi.first.math.util.Units;
//aaaAAAaAAaAaaaAAaAA
import frc.robot.Constants;
//aAaaaaaAAAaaaaaAAaA
import frc.robot.Constants.CANDevices;
//AaAaAaAaAaaaaAA

//AAAaaAAa
public class DriveAngle {
//AaAAaaaaA

//AAAAaaaaaAaAaaAAA
    private final Pigeon2 pigeon = new Pigeon2(CANDevices.pigeonIMU, Constants.canivoreName);
//AAAAA

//AaaaAaaaaAaAaAa
    private double degOffset = 0;
//AaaAAAA

//AaAAaAaaaAaAa
    public double headingRad;
//aAaaaAAAAAaaaaaA

//AAaaAAAAAAa
    public void updateHeading() {
//AAAAaAAAA
        headingRad = Units.degreesToRadians(pigeon.getYaw() - degOffset);
//Aaaaa
    }
//AAaAAAaaaaAAaAaa

//aaAaAaAaaAaAaaAAAaa
    public void resetHeading() {
//AaAaaAaaaAAAaaaaa
        degOffset = pigeon.getYaw();
//aaAAAAAaAaaA
    }  
//aaaaaa
    
//AAaAaaAAAaA
}