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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Ports.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

@Logged
public class AutoShooter extends SubsystemBase {
  // One motor is for starting the rollers, the other is for angling the shooter
  /** Creates a new Shooter. */
  private final TalonFX shooterMotor; // starting the rollers
  private final TalonFX angleMotor; // adjusting shooter to desired angle
  private final TalonFX indexerMotor;

  private final DutyCycleEncoder encoder;

  private final TalonFXConfiguration angleConfig;
  private final TalonFXConfiguration motorConfig;
  
  private final PIDController shooterPID;
  private final SimpleMotorFeedforward shooterFF;

  private final VoltageOut shooterVoltage;
  private final PositionVoltage angleVoltage;

  private final InterpolatingDoubleTreeMap velocityTable;
  private final InterpolatingDoubleTreeMap angleTable;

  private final SysIdRoutine shooterRoutine;

  private final Drive drive;

  private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0);
//   private final SimpleMotorFeedforward ff;

//   private final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
//   private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

  public AutoShooter(Drive drive) {
    shooterMotor = new TalonFX(ShooterPorts.SHOOTER_MOTOR, ShooterConstants.CANBUS);
    angleMotor = new TalonFX(ShooterPorts.ANGLE_MOTOR, ShooterConstants.CANBUS);
    indexerMotor = new TalonFX(ShooterPorts.INDEXER_MOTOR, ShooterConstants.CANBUS);
    
    velocityTable = new InterpolatingDoubleTreeMap();
    angleTable = new InterpolatingDoubleTreeMap();

    this.drive = drive;
    encoder = new DutyCycleEncoder(ShooterPorts.ENCODER);

    shooterVoltage = new VoltageOut(0);
    angleVoltage = new PositionVoltage(0);

    shooterFF = new SimpleMotorFeedforward(ShooterConstants.FFConstants.kS, ShooterConstants.FFConstants.kV);

    shooterPID = new PIDController(ShooterConstants.PIDConstants.kP, ShooterConstants.PIDConstants.kI, ShooterConstants.PIDConstants.kD);
    shooterPID.setTolerance(2.0);

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

    angleConfig.Slot0.kP = 5.0; 
    angleConfig.Slot0.kD = 0.1;

    // angleConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    // angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 

    motorConfig.Slot0.kP = ShooterConstants.PIDConstants.kP; 
    motorConfig.Slot0.kI = ShooterConstants.PIDConstants.kI;
    motorConfig.Slot0.kD = ShooterConstants.PIDConstants.kD;

    motorConfig.Slot0.kS = ShooterConstants.FFConstants.kS; 
    motorConfig.Slot0.kV = ShooterConstants.FFConstants.kV;

    shooterMotor.getVelocity().setUpdateFrequency(100);
    shooterMotor.getPosition().setUpdateFrequency(100);
    angleMotor.getPosition().setUpdateFrequency(100);

    shooterMotor.getConfigurator().apply(motorConfig);
    indexerMotor.getConfigurator().apply(motorConfig);

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

  public void autoShoot(double targetRPS, double targetRotations) {
      double currentRPS = shooterMotor.getVelocity().getValueAsDouble();

      double ff = shooterFF.calculate(targetRPS);
      double pid = shooterPID.calculate(currentRPS, targetRPS);

      shooterMotor.setControl(shooterVoltage.withOutput(ff + pid));
      angleMotor.setControl(angleVoltage.withPosition(targetRotations));
  }

  public boolean isReadyToShoot(double targetRPS) {
      return Math.abs(getShooterVelocity() - targetRPS) < 1.5;
  }

  public double getTargetRPS(double distance){
    return velocityTable.get(distance);
  }

  public double getTargetAngle(double distance){
    return angleTable.get(distance);
  }

  public void setShooterVelocity(double velocity) {
    shooterMotor.setControl(shooterVelocityControl.withVelocity(velocity));
  }

public Command autoShootSequence() {
    return this.run(() -> {
        double distance = drive.distanceFromTarget();
        double targetRPS = velocityTable.get(distance);
        double targetAngle = angleTable.get(distance);

        autoShoot(targetRPS, targetAngle);

        if (isReadyToShoot(targetRPS)) {
            indexerMotor.set(ShooterConstants.INDEXER_MOTOR_SPEED);
        } else {
            indexerMotor.set(0);
        }
    })
    .finallyDo(() -> {
        indexerMotor.set(0);
        shooterMotor.set(ShooterConstants.SHOOTER_CRUISE_SPEED);
    });
}

  public double getShooterVelocity(){
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public double getAngle(){
    return encoder.get();
  }

  public Command shootPIDtestCMD() {
    return this.run(() -> {
      setShooterVelocity(-15.0); 
      angleMotor.setControl(angleVoltage.withPosition(0.0497));
    });
  }

  public Command shootHubFlushCmd() {
    return this.run(() -> {
        setShooterVelocity(-55.0); 
        angleMotor.setControl(angleVoltage.withPosition(0.0497));
    });
  }

  public Command shootTrenchCmd() {
      return this.run(() -> {
          setShooterVelocity(-55.0); 
          angleMotor.setControl(angleVoltage.withPosition(0.97));
      });
  }

  public Command shootWallCmd() {
      return this.run(() -> {
          setShooterVelocity(-65.0);
          angleMotor.setControl(angleVoltage.withPosition(0.97));
      });
  }

  public Command runShooterMotorCmd(){
    return this.run(() -> shooterMotor.set(ShooterConstants.SHOOTER_MOTOR_SPEED));
  }

  public Command runShooterMotorPIDCmd() {
    return this.run(() -> setShooterVelocity(ShooterConstants.SHOOTER_MOTOR_SPEED));
  }
  
  public Command runIndexerMotorCmd(){
    return this.run(() -> indexerMotor.set(ShooterConstants.INDEXER_MOTOR_SPEED));
  }

  public Command runAngleMotorCmd(){
    return this.run(() -> angleMotor.set(ShooterConstants.ANGLE_MOTOR_SPEED));
  }

  public Command reverseAngleMotorCmd(){
    return this.run(() -> angleMotor.set(-ShooterConstants.ANGLE_MOTOR_SPEED));
  }

  //so we don't need to ramp up from rest every time
  public Command cruiseShooterMotorCmd(){
    return this.run(() -> shooterMotor.set(ShooterConstants.SHOOTER_CRUISE_SPEED));
  }

  public Command stopShooterMotorCmd() {
    return this.runOnce(() -> shooterMotor.set(0));
  }

   public Command stopAngleMotorCmd() {
    return this.runOnce(() -> angleMotor.set(0));
  }
   public Command stopIndexerMotorCmd() {
    return this.runOnce(() -> indexerMotor.set(0));
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
    // SmartDashboard.getNumber();

    double dist = drive.distanceFromTarget();
    double target = velocityTable.get(dist);
    
    SmartDashboard.putBoolean("READY TO FIRE", isReadyToShoot(target));
    SmartDashboard.putNumber("shooter velocity: ", getShooterVelocity());
    SmartDashboard.putNumber("shooter angle", getAngle());
    SmartDashboard.putNumber("distance from target", drive.distanceFromTarget());
  }
}
