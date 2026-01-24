package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class KaileyLeds extends SubsystemBase{
    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;

    public KaileyLeds() {
        lights = new AddressableLED(Ports.LedPorts.LED_PORT);
        buffer = new AddressableLEDBuffer(Constants.LedConstants.LED_LENGTH);
    }

}
