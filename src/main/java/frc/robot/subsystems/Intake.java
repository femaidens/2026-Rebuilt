// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports.*;
import frc.robot.subsystems.DriveConstants.Translation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */  
  private final TalonFX intakeMotor;
  private final TalonFX angleMotor;
  private final TalonFXConfiguration config;
  private final CANcoder encoder;
  private final MagnetSensorConfigs encoderConfig;
  private final PIDController anglePid; 

  private double setpoint;

  public Intake() {
    intakeMotor = new TalonFX(IntakePorts.INTAKE_MOTOR, IntakeConstants.CANBUS);
    angleMotor = new TalonFX(IntakePorts.ANGLE_MOTOR, IntakeConstants.CANBUS);

    encoder = new CANcoder(IntakePorts.CANCODER_ID, IntakeConstants.CANBUS);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(config);
    angleMotor.getConfigurator().apply(config);

    encoderConfig = new MagnetSensorConfigs();
    encoderConfig.withAbsoluteSensorDiscontinuityPoint(0.625); //225 degrees 
    encoder.getConfigurator().apply(encoderConfig.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    anglePid = new PIDController(IntakeConstants.PIDConstants.kP, IntakeConstants.PIDConstants.kI, IntakeConstants.PIDConstants.kP);
  }

  public Command setAnglePidCmd(double setpoint){
    return this.run(() -> setAnglePid(setpoint));
  }

  public Command setIntakeMotorCmd(){
    return this.run(() -> intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED));
  }

  public Command stopIntakeMotor(){
    return this.run(() -> intakeMotor.setVoltage(0));
  }

  public Command stopAngleMotor(){
    return this.run(() -> angleMotor.setVoltage(0));
  }

  public double getAngle(){
    return encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public void setAnglePid(double setpoint){
    angleMotor.setVoltage(anglePid.calculate(getAngle(), setpoint));
  }

  public boolean atAngle(double setpoint){
    return anglePid.atSetpoint();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
