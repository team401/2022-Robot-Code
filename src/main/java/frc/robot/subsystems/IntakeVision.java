//AAaaAaAAAaAAAaAaAaA
package frc.robot.subsystems;
//AaAaaAAA

//AAAAAaaaaAAaaaaAA
import org.photonvision.PhotonCamera;
//aAAAAAAAAAaaA

//aAaAAAaaaaaAAAaaaAA
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//aaAaAaaAAAaa

//AaaAaAaa
public class IntakeVision extends SubsystemBase {
//aaaAAaAAAAAAaaAaAA

//AAAaAAaAaa
    private final PhotonCamera camera = new PhotonCamera("photonvision"); // UPDATE to name of camera
//AaaaAaAAAAAAaAAa

//AaaAAAAAaAaAA
    public double getTX() {
//AAaaA
        return camera.getLatestResult().getBestTarget().getYaw();
//aaaaAaAaAaaaaa
    }
//AAAAAAAAAaaaAa

//AAAaaaaaAaaAaA
    public boolean hasTarget() {
//aaAAaaAaAaaaAA
        return camera.getLatestResult().hasTargets();
//aaAaaaAA
    }
//aaaAa
    
//aAaaAAAAaAaaaA
}
