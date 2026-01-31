package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;

public class KaileyLeds extends SubsystemBase {
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer;
    private int count = 0;

    public KaileyLeds() {
        strip = new AddressableLED(Ports.LedPorts.LED_PORT);
        buffer = new AddressableLEDBuffer(Constants.LedConstants.KAILEY_LED_LENGTH);

        strip.setLength(buffer.getLength());
        strip.setData(buffer);
        strip.start();
    }

    public void fillStrip(int g, int r, int b) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, g, r, b);
            //System.out.println("LED running");
        }
        strip.setData(buffer);
        // System.out.println("LED running");
    }

    public Command solidPurple() {
        return this.run(() -> {
            fillStrip(66, 104, 152);
            System.out.println("LED running purple");
        });
    }

    public Command solidGreen() {
        return this.run(() -> {
            fillStrip(212, 162, 10);
            System.out.println("LED running green");
        });
    }

    public Command clear() {
        return this.run(() -> {
            fillStrip(0, 0, 0);
        });
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
            System.out.println("LED running pulse");
        });
    }

    public Command sparkleEffect() {
        return this.run(() -> {
            fillStrip(66, 104, 152);
            for(int i = 0; i < 10; i++) {
                int position = (int) (Math.random() * buffer.getLength());
                buffer.setRGB(position, 212, 162, 10);
            }
            strip.setData(buffer);
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
        });
    }

    public Command offsetGradient() {
        return this.run(() -> {
            for(int i = 0; i < buffer.getLength(); i++) {
                int r = 0;
                int g = 0;
                int b = 0;
                if (i < buffer.getLength() / 2) {
                    g = (i * 255) / (buffer.getLength() / 2);
                    r = 66;
                    b = 104;
                }
                else {
                    r = 66;
                    b = 152;
                    g = ((i - (buffer.getLength() / 2)) * 255) / (buffer.getLength() / 2);
                }
                buffer.setRGB(i, r, g, b);
            }
            strip.setData(buffer);
        });
    }

    public Command greenGradient() {
        return this.run(() -> {
            for (int i = 0; i < buffer.getLength(); i++) {
                int g = (i * 255) / buffer.getLength();
                buffer.setRGB(i, 0, g / 2, 0);
            }
            strip.setData(buffer);
        });
    }

    public Command dimPurpleGradient() {
        return this.run(() -> {
            for (int i = 0; i < buffer.getLength(); i++) {
                int r = (i * 66) / buffer.getLength();
                int b = (i * 152) / buffer.getLength();
                buffer.setRGB(i, r / 2, 0, b / 2);
            }
            strip.setData(buffer);
        });
    }

    @Override
    public void periodic() {
        strip.setData(buffer);
    }

}