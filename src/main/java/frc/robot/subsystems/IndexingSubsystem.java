package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
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

    private final DigitalInput bottomBanner = new DigitalInput(DIOChannels.topBannerPort);
    private final DigitalInput topBanner = new DigitalInput(DIOChannels.bottomBannerPort);

    public IndexingSubsystem() {

        //ensure the intake motor stops when we don't command it to prevent jamming
        conveyorMotor.setNeutralMode(NeutralMode.Brake);

    }

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

}

