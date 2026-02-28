// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Ports.HopperPorts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
//adding
public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  private TalonFX indexMotor;
  private TalonFX spindexerMotor;
  // private final DigitalInput beambreak;
  
  public Hopper() {
    indexMotor = new TalonFX(HopperPorts.INDEX_MOTOR, HopperConstants.canbus);
    configureTalonMotor(indexMotor, HopperConstants.INDEXER_CURRENT_LIMIT, NeutralModeValue.Brake);
    spindexerMotor = new TalonFX(HopperPorts.HOPPER_MOTOR, HopperConstants.canbus);
    configureTalonMotor(spindexerMotor, HopperConstants.HOPPER_CURRENT_LIMIT, NeutralModeValue.Brake);
    // beambreak = new DigitalInput(HopperPorts.BEAM_BREAK);
  }

  //testing purposes
  public Hopper(TalonFX indexMotor, TalonFX spindexerMotor){
    this.indexMotor = indexMotor;
    this.spindexerMotor = spindexerMotor;
  }

  //Indexer motor
  public Command runIndexer(){
    return this.run(() -> indexMotor.set(HopperConstants.MOTORSPEED));
  }
  public Command stopIndexer(){
    return this.runOnce(() -> indexMotor.set(0));
  }

  
  public Command runSpindexer(){
    return this.run(() -> spindexerMotor.set(HopperConstants.MOTORSPEED));
  }
  public Command stopSpindexer(){
    return this.runOnce(() -> spindexerMotor.set(0));
  }
  /** 
  public boolean isFuelStaged(){
    return !beambreak.get();
  }
  */
  //Chat idk...configuring Talon
  public static void configureTalonMotor(TalonFX motor, double currentlimit, NeutralModeValue mode){
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentlimit;
    config.MotorOutput.NeutralMode = mode;

    motor.getConfigurator().apply(config);
  }
//change
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("FUEL STAGED", isFuelStaged());
  }
}
