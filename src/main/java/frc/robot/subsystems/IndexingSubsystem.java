package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.Constants.SuperstructureConstants;

public class IndexingSubsystem extends SubsystemBase {

    /**
     * For all our indexing needs :)
     * will probably be one motor and two banner sensors
     */
 
    private final WPI_TalonSRX conveyorMotor = new WPI_TalonSRX(CANDevices.conveyorMotorID);

    private final WPI_TalonSRX indexMotor = new WPI_TalonSRX(CANDevices.indexMotorID);

    private final DigitalInput bottomBanner = new DigitalInput(DIOChannels.topBannerPort);
    private final DigitalInput topBanner = new DigitalInput(DIOChannels.bottomBannerPort);

    public IndexingSubsystem() {

        //ensure the intake motor stops when we don't command it to prevent jamming
        conveyorMotor.setNeutralMode(NeutralMode.Brake);
        indexMotor.setNeutralMode(NeutralMode.Coast);

        conveyorMotor.setInverted(true);


    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("top banner state", getTopBannerState());
        SmartDashboard.putBoolean("bottom banner state", getBottomBannerState());

    }

    // True is tripped, False is not tripped
    public boolean getTopBannerState() {

        return !topBanner.get();

    }

    public boolean getBottomBannerState() {

        return !bottomBanner.get();

    }

    public void runConveyor() {

        conveyorMotor.set(SuperstructureConstants.conveyorPower);

    }

    public void runJoggingPower() {

        conveyorMotor.set(SuperstructureConstants.jogFowardPower);

    }

    public void stopConveyor() {

        conveyorMotor.set(0);

    }

    public void reverseConveyor() {

        conveyorMotor.set(-SuperstructureConstants.conveyorPower);

    }

    public void runIndexWheels() {

        indexMotor.set(-SuperstructureConstants.indexPower);

    }

    public void vomitWithIndexWheels() {

        indexMotor.set(2 * SuperstructureConstants.indexPower);

    }

    public void stopIndexWheels() {

        indexMotor.set(0.0);

    }

    public void reverseIndexWheels() {

        indexMotor.set(SuperstructureConstants.indexPower);

    }

}
