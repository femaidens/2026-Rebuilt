// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.CANBus;
//adding
import edu.wpi.first.epilogue.Logged;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

@Logged
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

    public static class HopperConstants{
        public static final double MOTORSPEED = 0.5;
        public static final double INDEXER_CURRENT_LIMIT = 30; 
        public static final double HOPPER_CURRENT_LIMIT = 40;
        public static final CANBus canbus = new CANBus("rio");
    }

  public static class IntakeConstants {
    public static final int CURRENT_LIMIT = 30;
    public static final double INTAKE_MOTOR_SPEED = 1.0;
    public static final CANBus CANBUS= new CANBus("rio");
    public static final double PIVOT_SPEED = 0.5;
    public static final double ANGLE_UP = 90; // random values for now until testing
    public static final double ANGLE_DOWN = 0; // same as above

      public static class PIDConstants {
      public static final double kP = 2;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    public static final int kOperatorControllerPort = 1;
  }

   public static class ShooterConstants{
    public static final int CURRENT_LIMIT = 30;
    public static final double INDEXER_MOTOR_SPEED = 0.3;
    public static final double SHOOTER_MOTOR_SPEED = -0.75;
    public static final double SHOOTER_CRUISE_SPEED = -0.05;
    public static final double ANGLE_MOTOR_SPEED = 0.1;

    public static final CANBus CANBUS = new CANBus("rio");
      
    public static class PIDConstants {
      public static final double kP = 0.1; //0.11 for speed 0.7 volts distance approx 150 angle 10
      public static final double kI = 0;
      public static final double kD = 0.0002; //0.0001
    }
    public static class FFConstants {
      public static final double kS = 0.13052;
      public static final double kV = 0.11939;
    }
  }
}
//pls let me merge