// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
/** Add your docs here. */
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.doubleThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith(MockitoExtension.class)
public class ShooterTest {
  @Mock
  private TalonFX shooterMotor;

  @Mock
  private TalonFX angleMotor;

  @Mock
  private PIDController anglePid;

  private Shooter shooter;

 @BeforeEach
  void setUp() {
    shooter = new Shooter(shooterMotor, angleMotor, anglePid);
  }

  @Test
  void runShooterMotor(){
    Command runShooterMotorCmd = shooter.runShooterMotorCmd();
    runShooterMotorCmd.initialize();
    runShooterMotorCmd.execute();

    verify(shooterMotor).set(Constants.ShooterConstants.SHOOTER_MOTOR_SPEED);
  }
//not finished yet
  @Test
  void setAngle(){
    Command setAngleCmd = shooter.setAngle(3.0);
    setAngleCmd.initialize();
    setAngleCmd.execute();

    assertEquals(setAngleCmd, setAngleCmd);
  }
}
