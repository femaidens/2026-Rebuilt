package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
// import com.ctre.lib.util.LEDStrip;

public class KaseyLeds extends SubsystemBase {
    private final AddressableLED leds; 
    private final AddressableLED buffer; //led buffer

    public KaseyLeds () {
        leds = new AddressableLED(Ports.LedPorts.KASEY_LED_PORT);
        buffer = new AddressableLEDBuffer (Constants.LedConstants.KASEY_LED_LENGTH);
        
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    public void setLEDGween() {
        for (var i = 0; i < buffer.getLength(); i++);
    }

}
