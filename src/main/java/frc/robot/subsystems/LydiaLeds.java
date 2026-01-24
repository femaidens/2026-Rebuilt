package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class LydiaLeds extends SubsystemBase {
    private final AddressableLED ledLight;
    private final AddressableLEDBuffer ledBuffer;

    public LydiaLeds() {
        ledLight = new AddressableLED(Ports.LedPorts.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.LED_LENGTH);

    }
    

}
