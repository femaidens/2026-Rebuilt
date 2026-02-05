// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Ports.*;
import frc.robot.Constants.*;

public class AutoShooter extends SubsystemBase {
  // One motor is for starting the rollers, the other is for angling the shooter
  /** Creates a new Shooter. */
  private final TalonFX shooterMotor; // starting the rollers
  private final TalonFX angleMotor; // adjusting shooter to desired angle

  private final CANcoder encoder;

  private final TalonFXConfiguration angleConfig;
  private final TalonFXConfiguration motorConfig;
  
  private final PIDController shooterPID;
  private final SimpleMotorFeedforward shooterFF;

  private final VoltageOut shooterVoltage;
  private final PositionVoltage angleVoltage;

  private final InterpolatingDoubleTreeMap velocityTable;
  private final InterpolatingDoubleTreeMap angleTable;

  private final SysIdRoutine shooterRoutine;
//   private final SimpleMotorFeedforward ff;

//   private final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
//   private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

  public AutoShooter() {
    shooterMotor = new TalonFX(ShooterPorts.SHOOTER_MOTOR, ShooterConstants.CANBUS);
    angleMotor = new TalonFX(ShooterPorts.ANGLE_MOTOR, ShooterConstants.CANBUS);
    
    velocityTable = new InterpolatingDoubleTreeMap();
    angleTable = new InterpolatingDoubleTreeMap();

    encoder = new CANcoder(ShooterPorts.CANCODER_ID);

    shooterVoltage = new VoltageOut(0);
    angleVoltage = new PositionVoltage(0);

    shooterFF = new SimpleMotorFeedforward(ShooterConstants.FFConstants.kS, ShooterConstants.FFConstants.kV);

    shooterPID = new PIDController(ShooterConstants.PIDConstants.kP, ShooterConstants.PIDConstants.kI, ShooterConstants.PIDConstants.kD);

    // angleConfig = new TalonFXConfiguration();
    // angleConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    // angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // motorConfig = new TalonFXConfiguration();
    // motorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    // motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // shooterMotor.getConfigurator().apply(motorConfig);
    // angleMotor.getConfigurator().apply(angleConfig);

    angleConfig = new TalonFXConfiguration();
    angleConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    angleConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

    // OPTIMIZE SIGNALS: Set frequencies for high-speed SysId 
    shooterMotor.getVelocity().setUpdateFrequency(100);
    shooterMotor.getPosition().setUpdateFrequency(100);
    angleMotor.getPosition().setUpdateFrequency(100);

    shooterMotor.getConfigurator().apply(motorConfig);
    angleMotor.getConfigurator().apply(angleConfig);

    setUpTables();

    shooterRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(), // Default ramp rates and step voltages
            new SysIdRoutine.Mechanism(
                // Voltage Setter: How SysId drives the motor
                (Voltage volts) -> shooterMotor.setControl(shooterVoltage.withOutput(volts.in(Units.Volts))),
                // Logger: How SysId records the motor's reaction
                log -> {
                    log.motor("shooter motor")
                        .voltage(Volts.of(shooterMotor.getMotorVoltage().getValueAsDouble()))
                        .angularPosition(Rotations.of(shooterMotor.getPosition().getValueAsDouble()))
                        .angularVelocity(RotationsPerSecond.of(shooterMotor.getVelocity().getValueAsDouble()));
                },
                this)
      );
  }

  public void setUpTables(){
    velocityTable.put(0.0, 0.0);
    
    angleTable.put(0.0, 0.0);
  }


  public Command runShooterMotorCmd(){
    return this.run(() -> shooterMotor.set(ShooterConstants.SHOOTER_MOTOR_SPEED));
  }

  //so we don't need to ramp up from rest every time
  public Command cruiseShooterMotorCmd(){
    return this.run(() -> shooterMotor.set(ShooterConstants.SHOOTER_CRUISE_SPEED));
  }

  public Command stopShooterMotorCmd() {
    return this.runOnce(() -> shooterMotor.set(0));
  }

  // quasi CMD
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return shooterRoutine.quasistatic(direction);
    }

  // dyna CMD
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return shooterRoutine.dynamic(direction);
    }
  
  @Override
  public void periodic() {
    // SmartDashboard.putBoolean(()); for once we get the angle 
  }
}
