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
        }
        strip.setData(buffer);
    }

    public Command solidPurple() {
        return this.run(() -> {
            fillStrip(66, 104, 152);
            System.out.println("LED Purple");
        });
    }

    public Command solidGreen() {
        return this.run(() -> {
            fillStrip(212, 162, 10);
            System.out.println("LED Green");
        });
    }

    public Command clear() {
        return this.run(() -> {
            fillStrip(0, 0, 0);
            System.out.println("LED Reset");
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

    @Override
    public void periodic() {
        strip.setData(buffer);
    }

}