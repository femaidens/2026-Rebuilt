// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.BeforeEach;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

@ExtendWith(MockitoExtension.class)
public class ClimbTest {

    @Mock
    private TalonFX mockLeaderMotor;

    @Mock
    private TalonFX mockFollowerMotor;

    @Mock
    private DigitalInput bottomLimitSwitch;

    @Mock
    private PIDController climbPID;

private Climb climb;

@BeforeEach
void setup(){
    climb = new Climb(mockLeaderMotor, mockFollowerMotor, bottomLimitSwitch,  climbPID);
}

@Test
    void reverseMotorCmd(){
        when(!bottomLimitSwitch.get()).thenReturn(false);

        Command reverseMotor = climb.reverseMotorCmd();
        reverseMotor.initialize();
        reverseMotor.execute();
        verify(mockLeaderMotor).set(0);
        verify(mockLeaderMotor).setPosition(0);
    }

@Test
void setLevel(){
    StatusSignal<Angle> fakeData = mock(StatusSignal.class);
    when(mockLeaderMotor.getPosition()).thenReturn(fakeData);
    when(fakeData.getValueAsDouble()).thenReturn(2.5);

    Command setLevelCmd = climb.setLevelCmd(29, true);
    setLevelCmd.initialize();
    setLevelCmd.execute();
    assertEquals(2.5,climb.getEncoderPosition());
    verify(climbPID).setPID(Constants.ClimbConstants.PIDConstants.kP_EXTEND,
    Constants.ClimbConstants.PIDConstants.kI_EXTEND,
    Constants.ClimbConstants.PIDConstants.kD_EXTEND
    ); 
    verify(climbPID).calculate(2.50,29);
    verify(mockLeaderMotor).setVoltage(anyDouble());
}

@Test
void climbPIDSet(){
    climb.climbPIDSet(false);
    verify(climbPID).setPID(Constants.ClimbConstants.PIDConstants.kP_RETRACT,
    Constants.ClimbConstants.PIDConstants.kI_RETRACT,
    Constants.ClimbConstants.PIDConstants.kD_RETRACT
    );
}
}
