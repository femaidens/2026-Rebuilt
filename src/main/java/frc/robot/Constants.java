// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix6.CANBus;

/** Add your docs here. */
public class Constants {

    public static class OperatorConstants{
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }
    public static class HopperConstants{
        public static final double MOTORSPEED = 0.5;
        public static final double INDEXER_CURRENT_LIMIT = 30; 
        public static final double HOPPER_CURRENT_LIMIT = 40;
        public static final CANBus canbus = new CANBus("rio");
    }
}
