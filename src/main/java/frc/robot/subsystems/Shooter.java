// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Port.ShooterPorts;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // One motor is for starting the rollers, the other is for angling the shooter
  /** Creates a new Shooter. */
  private final TalonFX shooterMotor; // starting the rollers
  private final TalonFX angleMotor; // adjusting shooter to desired angle
  private final CANcoder encoder;
  //private final TalonFXConfiguration angleConfig;
  private final TalonFXConfiguration motorConfig;
  private final PIDController shooterPID;
  private final SimpleMotorFeedforward shooterFF;

  public Shooter(TalonFX shooterMotor, TalonFX angleMotor, PIDController shooterPID, CANcoder encoder, SimpleMotorFeedforward shooterFF){
    this.shooterMotor = shooterMotor;
    this.angleMotor = angleMotor;
    this.shooterPID = shooterPID;
    this.encoder = encoder;
    this.shooterFF = shooterFF;
    motorConfig = new TalonFXConfiguration();

  }

  public Shooter() {
    shooterMotor = new TalonFX(ShooterPorts.SHOOTER_MOTOR, ShooterConstants.CANBUS);
    angleMotor = new TalonFX(ShooterPorts.ANGLE_MOTOR, ShooterConstants.CANBUS);
    encoder = new CANcoder(ShooterPorts.CANCODER_ID, ShooterConstants.CANBUS);
  
      // angleConfig = new TalonFXConfiguration();
      // angleConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
      // angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      motorConfig = new TalonFXConfiguration();
      motorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
      motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      shooterMotor.getConfigurator().apply(motorConfig);
      // angleMotor.getConfigurator().apply(angleConfig);
      shooterPID = new PIDController(
        Constants.ShooterConstants.PIDConstants.kP,
        Constants.ShooterConstants.PIDConstants.kI,
        Constants.ShooterConstants.PIDConstants.kD
      );
      shooterFF = new SimpleMotorFeedforward (
        Constants.ShooterConstants.FFConstants.kS,
        Constants.ShooterConstants.FFConstants.kV,
        Constants.ShooterConstants.FFConstants.kA);
  
    }

  public void shooterPID(double setpoint){
    angleMotor.setVoltage(shooterPID.calculate(setpoint, getCurrentPosition()));

  }

  

  public double getCurrentPosition(){
    return encoder.getPosition().getValueAsDouble()*360;
  }
  
  public Command runShooterMotorCmd(){
    return this.run(() -> shooterMotor.set(Constants.ShooterConstants.SHOOTER_MOTOR_SPEED));
  }

  public Command stopShooterMotorCmd() {
    return this.runOnce(() -> shooterMotor.set(0));
  }

   public Command setAngle(double setpoint) {
      return this.run(() -> shooterPID(setpoint));
    }


  @Override
  public void periodic() {
    // SmartDashboard.putBoolean(()); for once we get the angle 
  }
}
