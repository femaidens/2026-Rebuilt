// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ShooterConstants{
    public static final int CURRENT_LIMIT = 0;
    public static final double SHOOTER_MOTOR_SPEED = 0;
    public static final double ANGLE_MOTOR_SPEED = 0.7;
    public static CANBus CANBUS = new CANBus("rio");
      public static class PIDConstants {
        public static final double kP = 2;
        public static final double kI = 0;
        public static final double kD = 0;
     }
      public static class SetpointConstants {
        public static final double SMALL_ANGLE = 0;
        public static final double MIDDLE_ANGLE = 0;
        public static final double LARGE_ANGLE = 0;
      }
}
}
