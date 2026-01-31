// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.junit.Test;
import org.mockito.Mock;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class IntakeTest extends SubsystemBase {
  /** Creates a new IntakeTest. */
  private Intake intake;
  @ExtendWith(MockitoExtension.class)
  public IntakeTest() {
  @Mock
  TalonFX intakeMotor;
  @Mock
  TalonFX followerIntakeMotor;
  @Mock
  TalonFX angleMotor;
  @Mock
  TalonFXConfiguration angleConfig;
  @Mock
  TalonFXConfiguration motorConfig;
  @Mock
  CANcoder encoder;
  MagnetSensorConfigs encoderConfig;
  @Mock
  PIDController anglePid;
  }

  @BeforeEach
  void setUp() {
    intake = new Intake(intakeMotor, followerIntakeMotor, angleMotor, angleConfig, motorConfig, encoder, encoderConfig, anglePid);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach 
  void shutdown() throws Exception {
    intake.close(); 
  }

  @Test 
  void intakeFuel(){
    intake.setIntakeMotorCmd();
    assertEquals(
        0.5, intakeMotor.getSpeed(), DELTA);
    assertEquals(
        0.5, followerIntakeMotor.getSpeed(), DELTA);
  }

  @Test
  //initial position should be 90 degrees up
  void setIntakeInitialPos(){
    intake.setAnglePidCmd(90);
    assertEquals(
        90, angleMotor.getPosition() * 360, DELTA);
    assertEquals(
      true, anglePid.atSetpoint(), DELTA);
  }

  @Test
  void setIntakeDefaultPos(){
    intake.setAnglePidCmd(5);
    assertEquals(
        5, encoder.getAbsolutePosition().getValueAsDouble() * 360.0, DELTA);
    assertEquals(
      true, anglePid.atSetpoint(), DELTA);
  }
  @Test
  void stopAllMotors(){
    intake.setIntakeMotorCmd(0.5);
    intake.setAnglePidCmd(45);
    intake.stopAngleMotorCmd();
    intake.stopIntakeMotorCmd();
    assertEquals(
      0, angleMotor.getSpeed(), DELTA);
    assertEquals(
      0, intakeMotor.getSpeed(), DELTA);
     assertEquals(
      0, intakeFollowerMotor.getSpeed(), DELTA);   



  }
  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}
