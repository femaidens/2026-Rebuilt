package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Ports.LedPorts;

public class LEDS extends SubsystemBase {
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer;
    private int count = 0;
    private final Color firstRed, firstBlue, green, purple, black;

    public LEDS() {
        strip = new AddressableLED(Ports.LedPorts.LED_PORT);
        buffer = new AddressableLEDBuffer(Constants.LedConstants.LED_LENGTH);

        firstRed = Color.kGreen; //deep red (first)
        firstBlue = Color.kDarkViolet; //navy blue (first)
        green = Color.kFirstRed; //bright green
        purple = Color.kFirstBlue; //shows purple
        black = Color.kBlack; //off

        strip.setLength(buffer.getLength());
        strip.setData(buffer);
        strip.start();
    }

    public Command setDefault() {
        return this.run(() -> {
            LEDPattern off = LEDPattern.solid(black);
            off.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED Off");
        });
    }

    public Command setFirstRedSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(firstRed);
            solid.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED First Red");
        });
    }

    public Command setFirstBlueSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(firstBlue);
            solid.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED First Blue");
        });
    }

    public Command setGreenSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(green);
            solid.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED Green");
        });
    }

    public Command setPurpleSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(purple);
            solid.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED Purple");
        });
    }

    public void fillStrip(int g, int r, int b) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, g, r, b);
        }
        strip.setData(buffer);
    }

    public Command pulseEffect() {
        return this.run(() -> {
            count++;
            if(count < 40) {
                fillStrip(66, 104,152);
            }
            else if(count < 80) {
                fillStrip(212, 162, 10);
            }
            else {
                count = 0;
            }
            System.out.println("LED Pulse");
        });
    }

    public Command sparkleEffect() {
        return this.run(() -> {
            fillStrip(66, 104, 152);
            for(int i = 0; i < 50; i++) {
                int position = (int) (Math.random() * buffer.getLength());
                buffer.setRGB(position, 212, 162, 10);
            }
            strip.setData(buffer);
            System.out.println("LED Sparkle");
        });
    }

    public Command progressMask() {
        return this.run(() -> {
            double progress = (count % 100) / 100.0;
            for(int i = 0; i < buffer.getLength(); i++) {
                if(i < progress * buffer.getLength()) {
                    buffer.setRGB(i, 66, 104, 152);
                }
                else {
                    buffer.setRGB(i, 212, 162, 10);
                }
            }
            strip.setData(buffer);
            count++;
            System.out.println("LED Progress Mask");
        });
    }

    public Command offsetGradient() {
        return this.run(() -> {
            for (int i = 0; i < buffer.getLength(); i++) {
                int r = 0;
                int b = 0;

                if (i < buffer.getLength() / 2) {
                    r = (255 * i) / (buffer.getLength() / 2);
                    b = 255;
                }
                else {
                    b = (255 * (buffer.getLength() - i - 1)) / (buffer.getLength() / 2);
                    r = 255;
                }
                buffer.setRGB(i, 0, r, b);
            }
            strip.setData(buffer);
        });
    }

    public Command greenScroll() {
        return this.run(() -> {
            int index1 = count % buffer.getLength();
            for (int i = 0; i < buffer.getLength(); i++) {
                int index2 = (i + index1) % buffer.getLength();
                int g = (index2 * 255) / buffer.getLength();
                buffer.setRGB(i, g / 2, 0, 0);
            }
            strip.setData(buffer);
            System.out.println("LED Green Scroll");

            count++;
        });
    }

    public Command purpleScroll() {
        return this.run(() -> {
            int index1 = count % buffer.getLength();
            for (int i = 0; i < buffer.getLength(); i++) {
                int index2 = (i + index1) % buffer.getLength();
                int r = (index2 * 66) / buffer.getLength();
                int b = (index2 * 152) / buffer.getLength();
                buffer.setRGB(i, 0, r / 2, b / 2);
            }
            strip.setData(buffer);
            System.out.println("LED Purple Scroll");

            count++;
        });
    }

    public Command setFemaidensStaticGrad() {
        return this.run(() -> {
            LEDPattern femaidensGradient = LEDPattern.gradient(GradientType.kContinuous, green, purple);
            femaidensGradient.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED Femaidens Static Gradient");
        });
    }

    public Command setFemaidensScrollGrad() {
        return this.run(() -> {
            LEDPattern femaidensGradient = LEDPattern.gradient(GradientType.kContinuous, green, purple);
            LEDPattern scroll = femaidensGradient.scrollAtRelativeSpeed(Percent.per(Second).of(25));
            scroll.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED Femaidens Scroll Gradient");
        });
    }

    public Command setFemaidensPan() {
        return this.run(() -> {
            Map<Number, Color> maskSteps = Map.of(0, purple, 0.25, green, 0.5, black);
            LEDPattern femaidensGradient = LEDPattern.gradient(GradientType.kContinuous, green, purple);
            LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(15));
            
            LEDPattern pan = femaidensGradient.mask(mask);

            pan.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED Femaidens Pan Gradient");
        });
    }

    public Command setFemaidensSplit() {
        return this.run(() -> {
            LEDPattern femaidensSplit = LEDPattern.steps(Map.of(0, purple, 0.5, green));
            femaidensSplit.applyTo(buffer);
            strip.setData(buffer);
            System.out.println("LED Femaidens Split");
        });
    }

    
    @Override
    public void periodic() {
        strip.setData(buffer);
    }
    
}
