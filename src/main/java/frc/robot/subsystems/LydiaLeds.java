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

public class LydiaLeds extends SubsystemBase {
    private final AddressableLED ledLight;
    private final AddressableLEDBuffer ledBuffer;
    // private final AddressableLEDBufferView ledBufferViewLeft;
    // private final AddressableLEDBufferView ledBufferViewRight;

    private final Color firstRed, firstBlue, bumperRed, bumperBlue, green, purple, black;


    public LydiaLeds() {
        ledLight = new AddressableLED(Ports.LedPorts.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.LYDIA_LED_LENGTH);
        // ledBufferViewLeft = ledBuffer.createView(0,35);
        // ledBufferViewRight = ledBuffer.createView(35,69);

        //FYI - colors do not match the colors that appear on LEDS
        firstRed = Color.kGreen; //deep red (first)
        firstBlue = Color.kDarkViolet; //navy blue (first)
        bumperRed = Color.kDarkGreen; //red - not as bright
        bumperBlue = Color.kMediumBlue; //deep blue
        green = Color.kFirstRed; //bright green
        purple = Color.kFirstBlue; //shows purple
        black = Color.kBlack; //off


        ledLight.setLength(ledBuffer.getLength());

        ledLight.setData(ledBuffer);
        ledLight.start();

    }


    //Solid color Cmds
    public Command setDefault() {
        return this.run(() -> {
            LEDPattern off = LEDPattern.solid(black);
            off.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Off");
        });
    }

    public Command setFirstRedSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(firstRed);
            solid.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED First Red");
        });
    }

    public Command setFirstBlueSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(firstBlue);
            solid.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED First Blue");
        });
    }

    public Command setBumperRedSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(bumperRed);
            solid.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Bumper Red");
        });
    }

     public Command setBumperBlueSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(bumperBlue);
            solid.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Bumper Blue");
        });
    }

     public Command setGreenSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(green);
            solid.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Green");
        });
    }

     public Command setPurpleSolid() {
        return this.run(() -> {
            LEDPattern solid = LEDPattern.solid(purple);
            solid.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Purple");
        });
    }


    //Pattern color Cmds
    public Command setFemaidensStaticGrad() {
        return this.run(() -> {
            LEDPattern femaidensGradient = LEDPattern.gradient(GradientType.kContinuous, green, purple);
            femaidensGradient.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Femaidens Static Gradient");
        });
    }

    public Command setFemaidensScrollGrad() {
        return this.run(() -> {
            LEDPattern femaidensGradient = LEDPattern.gradient(GradientType.kContinuous, green, purple);
            LEDPattern scroll = femaidensGradient.scrollAtRelativeSpeed(Percent.per(Second).of(25));
            scroll.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Femaidens Scroll Gradient");
        });
    }

    public Command setFemaidensPan() {
        return this.run(() -> {
            Map<Number, Color> maskSteps = Map.of(0, purple, 0.25, green, 0.5, black);
            LEDPattern femaidensGradient = LEDPattern.gradient(GradientType.kContinuous, green, purple);
            LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(15));
            
            LEDPattern pan = femaidensGradient.mask(mask);

            pan.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Femaidens Pan Gradient");
        });
    }

    public Command setFemaidensSplit() {
        return this.run(() -> {
            LEDPattern femaidensSplit = LEDPattern.steps(Map.of(0, purple, 0.5, green));
            femaidensSplit.applyTo(ledBuffer);
            ledLight.setData(ledBuffer);
            System.out.println("LED Femaidens Split");
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
            // System.out.println("LED Femaidens Blink");
        });
    }
    


    @Override
    public void periodic() {
        ledLight.setData(ledBuffer);
    }

}
