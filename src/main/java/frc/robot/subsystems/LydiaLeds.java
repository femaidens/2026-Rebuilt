package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class LydiaLeds extends SubsystemBase {
    private final AddressableLED ledLight;
    private final AddressableLEDBuffer ledBuffer;
    // private final AddressableLEDBufferView ledBufferView;

    public LydiaLeds() {
        ledLight = new AddressableLED(Ports.LedPorts.LYDIA_LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.LYDIA_LED_LENGTH);
        // ledBufferView = new AddressableLEDBufferView(null, 0, 0);

        ledLight.setLength(ledBuffer.getLength());

        ledLight.setData(ledBuffer);
        ledLight.start();

    }
    
    @Override
    public void periodic() {
        ledLight.setData(ledBuffer);
    }

}
