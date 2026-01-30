package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class LydiaLeds extends SubsystemBase {
    private final AddressableLED ledLight;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView ledBufferView;

    private final LEDPattern firstRed;
    private final LEDPattern firstBlue;
    private final LEDPattern bumperRed;
    private final LEDPattern bumperBlue;
    private final LEDPattern green;
    private final LEDPattern purple;


    public LydiaLeds() {
        ledLight = new AddressableLED(Ports.LedPorts.LYDIA_LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.LYDIA_LED_LENGTH);
        ledBufferView = ledBuffer.createView(0,41);

        firstRed = LEDPattern.solid(Color.kFirstRed);
        firstBlue = LEDPattern.solid(Color.kFirstBlue);
        bumperRed = LEDPattern.solid(Color.kFirebrick);
        bumperBlue = LEDPattern.solid(Color.kMediumBlue);
        green = LEDPattern.solid(Color.kGreenYellow);
        purple = LEDPattern.solid(Color.kDarkViolet);


        ledLight.setLength(ledBuffer.getLength());

        ledLight.setData(ledBuffer);
        ledLight.start();

    }

    public Command setSolidColor() {
        return 
    }

    public void setFirstRed() {
        firstRed.applyTo(ledBuffer);
        ledLight.setData(ledBuffer);
    }

    public void setFirstBlue() {
        firstBlue.applyTo(ledBuffer);
        ledLight.setData(ledBuffer);
    }

    public void setBumperRed() {
        bumperRed.applyTo(ledBuffer);
        ledLight.setData(ledBuffer);
    }

     public void setBumperBlue() {
        bumperBlue.applyTo(ledBuffer);
        ledLight.setData(ledBuffer);
    }

     public void setGreen() {
        green.applyTo(ledBuffer);
        ledLight.setData(ledBuffer);
    }

     public void setPurple() {
        purple.applyTo(ledBuffer);
        ledLight.setData(ledBuffer);
    }
    
    @Override
    public void periodic() {
        ledLight.setData(ledBuffer);
        //highk dont know if i need this
    }

}
