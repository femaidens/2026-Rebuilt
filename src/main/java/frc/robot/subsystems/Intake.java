// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports.*;
import frc.robot.subsystems.DriveConstants.Translation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */  

  public final TalonFX intakeMotor;
  public final TalonFX angleMotor;
  public Intake() {


    intakeMotor = new TalonFX(IntakePorts.INTAKE_MOTOR, IntakeConstants.CANBUS.getName());
    angleMotor = new TalonFX(IntakePorts.ANGLE_MOTOR, IntakeConstants.CANBUS.getName());
  }

// HI IF U SEE THIS IM CURRENTLY FIGURING OUT THE CONFIGS PLS DONT REMOVE IT TY
  // public static void configureDriveTalon(TalonFX motor, int currentLimit) {
  //       TalonFXConfiguration config = new TalonFXConfiguration();
  //       config.CurrentLimits.SupplyCurrentLimit = currentLimit;
  //       config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  //       motor.getConfigurator().apply(config);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
