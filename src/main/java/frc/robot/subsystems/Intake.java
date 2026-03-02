// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */  
  private final TalonFX intakeMotor;
  private final TalonFX followerIntakeMotor;
  private final TalonFX angleMotor;
  private final TalonFXConfiguration angleConfig;
  private final TalonFXConfiguration motorConfig;
  private final DutyCycleEncoder encoder;
  private final PIDController anglePid; 

  public Intake() {
    intakeMotor = new TalonFX(IntakePorts.INTAKE_MOTOR, IntakeConstants.CANBUS);
    followerIntakeMotor = new TalonFX(IntakePorts.FOLLOWER_INTAKE_MOTOR, IntakeConstants.CANBUS);
    angleMotor = new TalonFX(IntakePorts.ANGLE_MOTOR, IntakeConstants.CANBUS);

    encoder = new DutyCycleEncoder(IntakePorts.ENCODER);

    angleConfig = new TalonFXConfiguration();
    angleConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    intakeMotor.getConfigurator().apply(motorConfig);
    followerIntakeMotor.getConfigurator().apply(motorConfig);
    angleMotor.getConfigurator().apply(angleConfig);
    

    followerIntakeMotor.setControl(new Follower(intakeMotor.getDeviceID(), MotorAlignmentValue.Aligned)); 

    anglePid = new PIDController(IntakeConstants.PIDConstants.kP, IntakeConstants.PIDConstants.kI, IntakeConstants.PIDConstants.kD);
    anglePid.setTolerance(0.05);

  }

  public Intake(TalonFX iM, TalonFX fM, TalonFX aM, DutyCycleEncoder e, PIDController p ){
    intakeMotor = iM;
    followerIntakeMotor = fM;
    angleMotor = aM;
    encoder = e;
    motorConfig = new TalonFXConfiguration();
    angleConfig = new TalonFXConfiguration();
    anglePid = p;
  }

  public Command setAngleUpDownCmd(){
      if(getAngle() <= IntakeConstants.ANGLE_UP - 10){
        return this.run(() -> setAnglePid(IntakeConstants.ANGLE_UP));
      }
      return this.run(() -> setAnglePid(IntakeConstants.ANGLE_DOWN));
  }

  public Command setAnglePidCmd(double setpoint){
    return this.run(() -> setAnglePid(setpoint));
  }

  public Command setAngleUpCmd(){
    return this.run(() -> intakeMotor.set(IntakeConstants.PIVOT_SPEED));
  }

  public Command setAngleDownCmd(){
    return this.run(() -> intakeMotor.set(-IntakeConstants.PIVOT_SPEED));
  }

  public Command setIntakeMotorCmd(){
    return this.run(() -> intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED));
  }

  public Command reverseIntakeMotorCmd(){
    return this.run(() -> intakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED));
  }

  public Command stopIntakeMotorCmd(){
    return this.run(() -> intakeMotor.setVoltage(0));
  }

  public Command stopAngleMotorCmd(){
    return this.run(() -> angleMotor.setVoltage(0));
  }

  public double getAngle(){
    return encoder.get() * 360.0;
  }

  public void setAnglePid(double setpoint){
    double voltage = anglePid.calculate(getAngle(), setpoint);
    angleMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public boolean atAngle(){
    return anglePid.atSetpoint();
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("At Correct Angle", atAngle());
    SmartDashboard.putNumber("Current Angle", getAngle());
  }
}
