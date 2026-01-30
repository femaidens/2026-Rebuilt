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
        strip = new AddressableLED(Ports.LedPorts.KAILEY_LED_PORT);
        buffer = new AddressableLEDBuffer(Constants.LedConstants.KAILEY_LED_LENGTH);

        strip.setLength(buffer.getLength());
        strip.setData(buffer);
        strip.start();
    }

    public Command fillStrip(int r, int g, int b) {
        return this.run(() -> {
            for(int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, r, g, b);
            }
            strip.setData(buffer);
        });
    }

    public Command solidPurple() {
        return this.run(() -> {
            fillStrip(104, 66, 152);
        });
    }

    public Command solidGreen() {
        return this.run(() -> {
            fillStrip(162, 212, 10);
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
                solidPurple();
            }
            else if(count < 80) {
                solidGreen();
            }
            count = 0;
        });
    }

    public Command sparkleEffect() {
        return this.run(() -> {
            fillStrip(104, 66, 152);
            for(int i = 0; i < 10; i++) {
                int position = (int) (Math.random() * buffer.getLength());
                buffer.setRGB(position, 162, 212, 10);
            }
            strip.setData(buffer);
        });
    }

    @Override
    public void periodic() {
        strip.setData(buffer);
        pulseEffect().schedule();
    }

}