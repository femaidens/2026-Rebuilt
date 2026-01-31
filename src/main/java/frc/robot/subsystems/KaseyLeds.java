package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Ports.LedPorts;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import com.ctre.lib.util.LEDStrip;
import edu.wpi.first.wpilibj.LEDReader;

public class KaseyLeds extends SubsystemBase {
    private final AddressableLED leds; 
    private final AddressableLEDBuffer buffer; //led buffer

    // private final Color red, blue, green, purple, pink

    public KaseyLeds () {
        leds = new AddressableLED(Ports.LedPorts.KASEY_LED_PORT);
        buffer = new AddressableLEDBuffer (Constants.LedConstants.KASEY_LED_LENGTH);
        
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }
    
    public void setLEDPurple() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB (i, 165, 92, 255);
        }
        leds.setData(buffer);
    }
    
    public void setLEDGreen() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 104, 255, 84);
        }
        leds.setData(buffer);
    }

  public void setLEDRed() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB (i, 255, 0, 0);
        }
        leds.setData(buffer);
    }

    public void setLEDBlue() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB (i, 0, 221, 255);
        }
        leds.setData(buffer);
    }

    
    public void setLEDPink() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB (i, 255, 74, 158);
        }
        leds.setData(buffer);
    }


    public Command setDefault() {
        return this.run(() -> {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB (i, 0, 0 ,0);
            }
            leds.setData(buffer);
        });
    }

    public Command setPurpleCommand() {
        return this.run(() -> setLEDPurple());
        }

    public Command setGreenCommand() {
        return this.run(() -> setLEDGreen());
    }

    public Command setRedCommand() {
        return this.run(() -> setLEDRed());
    }

    public Command setBlueCommand() {
        return this.run(() -> setLEDBlue());
    }

    public Command setPinkCommand() {
        return this.run(() -> setLEDPink());
    }

    //beautiful transitions
    public Command breatheEffect() {
        return this.run(() -> {
            LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
            LEDPattern pattern = base.breathe(Seconds.of(2));


            pattern.applyTo(buffer);
            leds.setData(buffer);
        });
    }

    @Override
    public void periodic() {
        leds.setData(buffer);
        breatheEffect().schedule();
    }

}




