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
    private boolean movingForward;
    private double currentProgress; // [0,1]
    private static final double progressPerTick = 0.01;
    
    
        // private final Color red, blue, green, purple, pink
    
        public KaseyLeds () {
            leds = new AddressableLED(Ports.LedPorts.LED_PORT);
            buffer = new AddressableLEDBuffer (Constants.LedConstants.KASEY_LED_LENGTH);
            
            leds.setLength(buffer.getLength());
            leds.setData(buffer);
            leds.start();
    
            movingForward = true;
            currentProgress = 0.0;
        }
        
        public void setLEDPurple() {
            for (var i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB (i, 92, 165, 255);
            }
            leds.setData(buffer);
        }
        
        public void setLEDGreen() {
            for (var i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 255, 104, 84);
            }
            leds.setData(buffer);
        }
    
      public void setLEDRed() {
            for (var i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB (i, 0, 255, 0);
            }
            leds.setData(buffer);
        }
    
        public void setLEDBlue() {
            for (var i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB (i, 221, 0, 255);
            }
            leds.setData(buffer);
        }
    
        
        public void setLEDPink() {
            for (var i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB (i, 74, 255, 158);
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
                LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, new Color (0, 255, 0), new Color(221, 0, 255));
                LEDPattern pattern = base.breathe(Seconds.of(2));
    
    
                pattern.applyTo(buffer);
                leds.setData(buffer);
            });
        }
    
        public double getLedProgress () {
            return currentProgress;
        }
    
        public Command progressMaskEffect() {
            return this.run(() -> {
            LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, new Color (0, 255, 0), new Color (0, 0, 255));
            LEDPattern mask = LEDPattern.progressMaskLayer(this::getLedProgress);
            LEDPattern heightDisplay = base.mask(mask);
    
            heightDisplay.applyTo(buffer);
    
            leds.setData(buffer);
          System.out.println(this.getLedProgress());
        });
    }
    
    public Command progressMaskEffectGreenPurple() {
        return this.run(() -> {
        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, new Color (92, 165, 255), new Color (255, 104, 84));
        LEDPattern mask = LEDPattern.progressMaskLayer(this::getLedProgress);
        LEDPattern heightDisplay = base.mask(mask);

        heightDisplay.applyTo(buffer);

        leds.setData(buffer);
    });
}

        @Override
        public void periodic() {
            leds.setData(buffer);
          //  breatheEffect().schedule();
    
            if(movingForward) {
                currentProgress += progressPerTick;
            } else {
                currentProgress -= progressPerTick;
            }
            if (currentProgress>=1.0 || currentProgress<=0.0) {
                movingForward = !movingForward;
            }
    
        }
    }




