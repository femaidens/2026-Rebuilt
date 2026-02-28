// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
//adding
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
public class HopperTest {

    @Mock
    private TalonFX indexMotor;
    @Mock
    private TalonFX spindexerMotor;

    private Hopper hopper;

    @BeforeEach
    void setUp() {
        hopper = new Hopper(indexMotor, spindexerMotor);
    }

    @Test
    void runMotors(){
        Command runIndexer = hopper.runIndexer();
        runIndexer.initialize();
        runIndexer.execute();

        Command runSpindexer = hopper.runSpindexer();
        runSpindexer.initialize();
        runSpindexer.execute();
        
        verify(indexMotor).set(HopperConstants.MOTORSPEED);
        verify(spindexerMotor).set(HopperConstants.MOTORSPEED);

    }

    @Test
    void stopMotors(){
        Command stopIndexer = hopper.stopIndexer();
        stopIndexer.initialize();
        stopIndexer.execute();

        Command stopSpindexer = hopper.stopSpindexer();
        stopSpindexer.initialize();
        stopSpindexer.execute();
        
        verify(indexMotor).set(0);
        verify(spindexerMotor).set(0);

    }

}
