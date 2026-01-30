package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
//import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import com.ctre.lib.util.LEDStrip;

public class KaseyLeds extends SubsystemBase {
    private final AddressableLED leds; 
    private final AddressableLEDBuffer buffer; //led buffer

    // private final Color red, blue, green, purple, pink


    //idk if i made it too long and complicated --> reminder to maybe change stuff idk
    public KaseyLeds () {
        leds = new AddressableLED(Ports.LedPorts.KASEY_LED_PORT);
        buffer = new AddressableLEDBuffer (Constants.LedConstants.KASEY_LED_LENGTH);
        
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }
    
    public void setLEDPurple() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB (i, 207, 159, 255);
        }
        leds.setData(buffer);
    }
    
    public void setLEDGreen() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 148, 217, 590);
        }
        leds.setData(buffer);
    }
    public void setDefault() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0 ,0);
        }
        leds.setData(buffer);
    }

    public void setSolidBlack(int[] solid) {
        for (int k = 0; k < buffer.getLength(); k++) {
            buffer.setRGB(k,solid[0], solid[1], solid[2]);
        }
        leds.setData(buffer);
        System.out.println("wow its a solid");
    }

    @Override
    public void periodic() {
        leds.setData(buffer);
    }

}
