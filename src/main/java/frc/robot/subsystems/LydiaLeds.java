package frc.robot.subsystems;

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

public class LydiaLeds extends SubsystemBase {
    private final AddressableLED ledLight;
    private final AddressableLEDBuffer ledBuffer;
    // private final AddressableLEDBufferView ledBufferViewLeft;
    // private final AddressableLEDBufferView ledBufferViewRight;

    private final Color firstRed, firstBlue, bumperRed, bumperBlue, green, purple, black;


    public LydiaLeds() {
        ledLight = new AddressableLED(Ports.LedPorts.LYDIA_LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.LYDIA_LED_LENGTH);
        // ledBufferViewLeft = ledBuffer.createView(0,35);
        // ledBufferViewRight = ledBuffer.createView(35,69);

        firstRed = Color.kFirstRed;
        firstBlue = Color.kFirstBlue;
        bumperRed = Color.kFirebrick;
        bumperBlue = Color.kMediumBlue;
        green = Color.kGreenYellow;
        purple = Color.kDarkViolet;
        black = Color.kBlack;


        ledLight.setLength(ledBuffer.getLength());

        ledLight.setData(ledBuffer);
        ledLight.start();

    }


    //Solid color Cmds
    public Command setDefault() {
        return this.run(() -> {
            LEDPattern off = LEDPattern.solid(black);
            off.applyTo(ledBuffer);
        });
    }

    public Command setFirstRedSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(firstRed);
            solid.applyTo(ledBuffer);
        });
    }

    public Command setFirstBlueSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(firstBlue);
            solid.applyTo(ledBuffer);
        });
    }

    public Command setBumperRedSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(bumperRed);
            solid.applyTo(ledBuffer);
        });
    }

     public Command setBumperBlueSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(bumperBlue);
            solid.applyTo(ledBuffer);
        });
    }

     public Command setGreenSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(green);
            solid.applyTo(ledBuffer);
        });
    }

     public Command setPurpleSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(purple);
            solid.applyTo(ledBuffer);
        });
    }


    //Pattern color Cmds
    public Command setFemaidensGrad() {
        return this.run(() -> {
            LEDPattern femaidensGradient = LEDPattern.gradient(GradientType.kContinuous, green, purple);
            femaidensGradient.applyTo(ledBuffer);
        });
    }

    public Command setFemaidensSplit() {
        return this.run(() -> {
            LEDPattern femaidensSplit = LEDPattern.steps(Map.of(0, purple, 0.5, green));
            femaidensSplit.applyTo(ledBuffer);
        });
    }

    public Command setFemaidensBlink() {
        return this.run(() -> {
            LEDPattern solidPurple = LEDPattern.solid(purple);
            LEDPattern solidGreen = LEDPattern.solid(green);
            LEDPattern off = LEDPattern.solid(black);

            while (true) {
                solidPurple.applyTo(ledBuffer);
                off.applyTo(ledBuffer);
                solidGreen.applyTo(ledBuffer);
            }
        });
    }
    


    @Override
    public void periodic() {
        ledLight.setData(ledBuffer);
    }

}
