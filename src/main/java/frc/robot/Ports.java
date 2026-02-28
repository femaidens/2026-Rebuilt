package frc.robot;

public class Ports {
    
    public static final class LedPorts {
        public static final int LED_PORT = 3;
    }
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/** Add your docs here. */
        public class DrivetrainPorts {
        public static final int FRONT_LEFT_DRIVE = 13;
        public static final int REAR_LEFT_DRIVE = 11;
        public static final int FRONT_RIGHT_DRIVE = 12;
        public static final int REAR_RIGHT_DRIVE = 10;
    
        public static final int FRONT_LEFT_TURN = 9;
        public static final int REAR_LEFT_TURN = 15;
        public static final int FRONT_RIGHT_TURN = 4;
        public static final int REAR_RIGHT_TURN = 20;

        public static final int FRONT_LEFT_CANCODER = 0;
        public static final int FRONT_RIGHT_CANCODER = 2;
        public static final int REAR_LEFT_CANCODER = 3;
        public static final int REAR_RIGHT_CANCODER = 1;

          public static final int GYRO_ID = 8; 
    }

    public class ShooterPorts {
        public static final int SHOOTER_MOTOR  = 0;
        public static final int ANGLE_MOTOR = 0;
        public static final int CANCODER_ID = 0;
    }
     
    public class HopperPorts{
        public static final int INDEX_MOTOR = 0; 
        public static final int BEAM_BREAK = 1;
        public static final int HOPPER_MOTOR = 2;
    }

    public class IntakePorts {
        public static final int INTAKE_MOTOR = 0;
        public static final int ANGLE_MOTOR = 0;
        public static final int CANCODER_ID = 0;
        public static final int FOLLOWER_INTAKE_MOTOR = 0;
    }
}
