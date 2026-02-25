// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    public static final int OPERATOR_PORT = 1;
  }

  public static class ClimbConstants {
    public static final double MOTOR_SPEED = 0.1;
    public static final CANBus CAN_BUS = new CANBus("rio");

    public static class PIDConstants {
		  public static final double kP_EXTEND = 6.7;
		  public static final double kI_EXTEND = 6.7;
		  public static final double kD_EXTEND = 6.7;
		  public static final double kP_RETRACT = 2.1;
		  public static final double kI_RETRACT = 2.1;
		  public static final double kD_RETRACT = 2.1;
    }

    public static class SetpointConstants {
      public static final double MINIMUM = 0;
      public static final double LEVEL_ONE = 27;
      public static final double LEVEL_TWO_THREE = 18;
    }
    public static final int OPERATOR_PORT = 0;
        public static final int DRIVER_PORT = 1;

  }

  public static class LedConstants {
    public static final int LYDIA_LED_LENGTH = 177;
    public static final int KASEY_LED_LENGTH = 177;
    public static final int KAILEY_LED_LENGTH = 177;
    public static final int LED_LENGTH = 177;
    public static final int DRIVER_PORT = 0;
  }

  public static class IntakeConstants {
    public static final int CURRENT_LIMIT = 30;
    public static final double INTAKE_MOTOR_SPEED = 0.5;
    public static final CANBus CANBUS= new CANBus("rio");
    public static final double PIVOT_SPEED = 0.5;
    public static final double ANGLE_UP = 90; // random values for now until testing
    public static final double ANGLE_DOWN = 0; // same as above

      public static class PIDConstants {
      public static final double kP = 2;
      public static final double kI = 0;
      public static final double kD = 0;
    }

  }

  public static class ClimbL1Constants {
    public static final double MOTOR_SPEED = 0.1;
    public static final CANBus CAN_BUS = new CANBus("rio");

    public static class PIDConstants {
		  public static final double kP_EXTEND = 6.7;
		  public static final double kI_EXTEND = 6.7;
		  public static final double kD_EXTEND = 6.7;
		  public static final double kP_RETRACT = 2.1;
		  public static final double kI_RETRACT = 2.1;
		  public static final double kD_RETRACT = 2.1;
    }

    public static class SetpointConstants {
      public static final double MINIMUM = 0;
      public static final double LEVEL_ONE = 27;
    }
  }

}