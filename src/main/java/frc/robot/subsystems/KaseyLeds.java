package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.feedback.led.Color;
// import com.ctre.lib.feedback.led.LEDStrip;

public class KaseyLeds {
    private final int BUFFER_LENGTH = 67;
    private final AddressableLED leds; 
    private final AddressableLED buffer; //led buffer

    public KaseyLeds () {
        leds = new AddressableLED();

    }
}
