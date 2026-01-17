package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;

public class Climb extends SubsystemBase{
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final DigitalInput bottomLimitSwitch;
    private final PIDController climbPID;
    private final RelativeEncoder climbEncoder;




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






}