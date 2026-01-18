package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final DigitalInput bottomLimitSwitch;
    private final PIDController climbPID;
    private final AbsoluteEncoder climbEncoder;

    public Climb () {
    // Initializing fields --> class Name = new smth();
        Climb leaderMotor = new TalonFX();
        Climb followerMotor = new TalonFX();
        Climb bottomLimitSwitch = new DigitalInput();
        Climb DigitalInput = new PIDController();
        Climb climbEncoder = new RelativeEncoder(); 

    }

    public Command runMotor() {
        return this.run(() -> leaderMotor.set(Constants.ClimbConstants.MOTOR_SPEED));
    }
    
    public Command reverseMotorCmd() {
        return this.run(() -> {
            if(hitBottomLimit()) {
                leaderMotor.set(0);
                climbEncoder.setPosition(0);
            } else {
                leaderMotor.set(-Constants.ClimbConstants.MOTOR_SPEED);
            }

        });
    }

    public boolean hitBottomLimit() {
        return !bottomLimitSwitch.get();
    }

    // public Command setLevel(double setpoint) {
    //     return this.run(() -> climbPID)
    // } 

    public double getEncoderPosition() {
        return climbEncoder.getPosition();
    }

    public void setVoltage(double v) {
        leaderMotor.setVoltage(v);
    }


}