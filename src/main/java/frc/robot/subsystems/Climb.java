package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Ports;

public class Climb extends SubsystemBase {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final DigitalInput bottomLimitSwitch;
    private final PIDController climbPID;

    public Climb () {
        leaderMotor = new TalonFX(Ports.ClimbPorts.LEADER_MOTOR, new CANBus());
		followerMotor = new TalonFX(Ports.ClimbPorts.FOLLOWER_MOTOR, new CANBus());
        bottomLimitSwitch = new DigitalInput(Ports.ClimbPorts.BOTTOM_LIMIT_SWITCH);
        climbPID = new PIDController(0,0,0);
    }

    public Command runMotor() {
        return this.run(() -> leaderMotor.set(Constants.ClimbConstants.MOTOR_SPEED));
    }
    
    public Command reverseMotorCmd() {
        return this.run(() -> {
            if(hitBottomLimit()) {
                leaderMotor.set(0);
                leaderMotor.setPosition(0);
            } else {
                leaderMotor.set(-Constants.ClimbConstants.MOTOR_SPEED);
            }

        });
    }

    public Command stopMotorCmd() {
        return this.runOnce(() -> leaderMotor.set(0));
    }

    public boolean readyToLift() {
        if((getEncoderPosition() == 27) || (getEncoderPosition() == 45) || (getEncoderPosition() == 63)) {
            return true;
        } else {
            return false;
        }
    }

    public void climbPIDSet(double setpoint, boolean readyToLift) {
	    if(readyToLift) {
		    climbPID.setPID(
                Constants.ClimbConstants.PIDConstants.kP_EXTEND,
                Constants.ClimbConstants.PIDConstants.kI_EXTEND,
                Constants.ClimbConstants.PIDConstants.kD_EXTEND
            );
	    } else {
		    climbPID.setPID(
                Constants.ClimbConstants.PIDConstants.kP_RETRACT,
                Constants.ClimbConstants.PIDConstants.kI_RETRACT,
                Constants.ClimbConstants.PIDConstants.kD_RETRACT
            );
	    }
    }

    public void climbPIDController(double current, double setpoint) {
	    leaderMotor.setVoltage(climbPID.calculate(leaderMotor.getPosition().getValueAsDouble(), setpoint));
    }

    public boolean hitBottomLimit() {
        return !bottomLimitSwitch.get();
    }

    // public Command setLevel(double setpoint) {
    //     return this.run(() -> climbPID)
    // } 
    
    public Command resetEncoderCmd() {
        return this.runOnce(() -> leaderMotor.setPosition(0));
    }

    public double getEncoderPosition() {
        return leaderMotor.getPosition().getValueAsDouble();
    }

    public void setVoltage(double v) {
        leaderMotor.setVoltage(v);
    }


}