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

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  private final TalonFX indexMotor;
  private final TalonFXConfiguration config;
  private final DigitalInput beambreak;
  
  public Hopper() {
    indexMotor = new TalonFX(HopperPorts.INDEX_MOTOR);
    config = new TalonFXConfiguration();
    beambreak = new DigitalInput(HopperPorts.BEAM_BREAK);
  }

  public Command runMotor(){
    return this.run(() -> indexMotor.set(HopperConstants.MOTORSPEED));
  }
  public Command stopMotor(){
    return this.runOnce(() -> indexMotor.set(0));
  }
  public boolean isFuelStaged(){
    return !beambreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("FUEL STAGED", isFuelStaged());
  }
}
