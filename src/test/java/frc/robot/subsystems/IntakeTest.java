// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

@ExtendWith(MockitoExtension.class)
public class IntakeTest {
  /** Creates a new IntakeTest. */
  private Intake intake;

  @Mock
  private TalonFX intakeMotor;

  @Mock
  private TalonFX followerIntakeMotor;

  @Mock
  private TalonFX angleMotor;

  @Mock
  private CANcoder encoder;

  @Mock
  private PIDController anglePid;

  @BeforeEach
  void setUp() {
    intake = new Intake(intakeMotor, followerIntakeMotor, angleMotor, encoder, anglePid);
  }

  @Test
  void intakeFuel() {
    Command cmd = intake.setIntakeMotorCmd();

    cmd.initialize();
    cmd.execute(); 

    verify(intakeMotor).set(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  @Test
  void notAtCorrectPos(){
    StatusSignal<Angle> fakeData = mock(StatusSignal.class);
    when(encoder.getAbsolutePosition()).thenReturn(fakeData);
    when(fakeData.getValueAsDouble()).thenReturn(0.0);
    when(anglePid.atSetpoint()).thenReturn(false);

    Command setAnglePid = intake.setAnglePidCmd(20);
    setAnglePid.initialize();
    setAnglePid.execute(); 

    assertEquals(0, intake.getAngle(), 0.001);
    assertFalse(intake.atAngle(), "Angle should not be at setpoint");
    
    verify(angleMotor).setVoltage(anyDouble());
    verify(anglePid).calculate(0,20);
  }

  @Test
  void setIntakeInitialPos() {
    StatusSignal<Angle> fakeData = mock(StatusSignal.class);
    when(encoder.getAbsolutePosition()).thenReturn(fakeData);
    when(fakeData.getValueAsDouble()).thenReturn(0.25);
    when(anglePid.atSetpoint()).thenReturn(true);

    Command setAnglePid = intake.setAnglePidCmd(90);
    setAnglePid.initialize();
    setAnglePid.execute(); 

    assertEquals(90.0, intake.getAngle(), 0.001);
    assertTrue(intake.atAngle(), "Angle should be at setpoint");
    
    verify(angleMotor).setVoltage(anyDouble());
    verify(anglePid).calculate(90,90);

  }
}