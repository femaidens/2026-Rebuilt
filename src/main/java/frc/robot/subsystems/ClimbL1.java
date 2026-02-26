package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

@Logged
public class ClimbL1 extends SubsystemBase {
    
    //might have to add a field for CANBus - yet to be tested
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final DigitalInput bottomLimitSwitch;
    private final PIDController climbPIDExtend;
    private final PIDController climbPIDRetract;
    
    public ClimbL1(TalonFX leaderMotor, TalonFX followerMotor,DigitalInput bottomLimitSwitch, PIDController climbPIDExtend, PIDController climbPIDRetract) {
        this.leaderMotor = leaderMotor;
        this.followerMotor = followerMotor;
        this.bottomLimitSwitch = bottomLimitSwitch;
        this.climbPIDExtend = climbPIDExtend;
        this.climbPIDRetract = climbPIDRetract;
    }


    public ClimbL1 () {
        leaderMotor = new TalonFX(Ports.ClimbPorts.LEADER_MOTOR, Constants.ClimbConstants.CAN_BUS);
		followerMotor = new TalonFX(Ports.ClimbPorts.FOLLOWER_MOTOR, Constants.ClimbConstants.CAN_BUS);
        bottomLimitSwitch = new DigitalInput(Ports.ClimbPorts.BOTTOM_LIMIT_SWITCH);
        climbPIDExtend = new PIDController
                (Constants.ClimbConstants.PIDConstants.kP_EXTEND,
                Constants.ClimbConstants.PIDConstants.kI_EXTEND,
                Constants.ClimbConstants.PIDConstants.kD_EXTEND);
        climbPIDExtend.setTolerance(0.21);
        climbPIDRetract = new PIDController
                (Constants.ClimbConstants.PIDConstants.kP_RETRACT,
                Constants.ClimbConstants.PIDConstants.kI_RETRACT,
                Constants.ClimbConstants.PIDConstants.kD_RETRACT);
        climbPIDRetract.setTolerance(0.21);

        Follower follower = new Follower(Ports.ClimbPorts.LEADER_MOTOR, MotorAlignmentValue.Aligned);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        //configs not finalized - subject to change
            configs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
            configs.CurrentLimits.withSupplyCurrentLimit(30);

        leaderMotor.getConfigurator().apply(configs);
        followerMotor.getConfigurator().apply(configs);
        followerMotor.setControl(follower);

    }

    public Command runMotorCmd() {
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

    // public void climbPIDSet(boolean readyToLift) {
	//     if(readyToLift) {
	// 	    climbPID.setPID(
    //             Constants.ClimbL1Constants.PIDConstants.kP_EXTEND,
    //             Constants.ClimbL1Constants.PIDConstants.kI_EXTEND,
    //             Constants.ClimbL1Constants.PIDConstants.kD_EXTEND
    //         );
	//     } else {
	// 	    climbPID.setPID(
    //             Constants.ClimbL1Constants.PIDConstants.kP_RETRACT,
    //             Constants.ClimbL1Constants.PIDConstants.kI_RETRACT,
    //             Constants.ClimbL1Constants.PIDConstants.kD_RETRACT
    //         );
	//     }
    // }

    public void climbPIDControllerExtend(double setpoint) {
	    leaderMotor.setVoltage(climbPIDExtend.calculate(leaderMotor.getPosition().getValueAsDouble(), setpoint));
    }

    public void climbPIDControllerRetract(double setpoint) {
	    leaderMotor.setVoltage(climbPIDRetract.calculate(leaderMotor.getPosition().getValueAsDouble(), setpoint));
    }

    public Command setLevelCmd(double setpoint) {
        return this.run(() -> {
            climbPIDControllerExtend(setpoint);
        });
    }

    public Command retractLevelCmd(double setpoint) {
        return this.run(() -> {
            climbPIDControllerRetract(setpoint);
        });
    }

    public boolean hitBottomLimit() {
        return !bottomLimitSwitch.get();
    }
    
    public Command resetEncoderCmd() {
        return this.runOnce(() -> leaderMotor.setPosition(0));
    }

    public double getEncoderPosition() {
        return leaderMotor.getPosition().getValueAsDouble();
    }

    public void setVoltage(double v) {
        leaderMotor.setVoltage(v);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hit Bottom Limit", hitBottomLimit());
        SmartDashboard.putNumber("Encoder Position", getEncoderPosition());
    }


}
