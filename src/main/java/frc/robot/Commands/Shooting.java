// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterConstants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shooting{

  //update with vision later

  private Shooter shooter;

  public Shooting(){
    this.shooter = new Shooter();
  }

  public Command smallAngleCmd(){
    return shooter.setAngle(ShooterConstants.shootConstants.SetpointConstants.SMALL_ANGLE);
  }
  
  public Command middleAngleCmd(){
    return shooter.setAngle(ShooterConstants.shootConstants.SetpointConstants.MIDDLE_ANGLE);
  }

  public Command largeAngleCmd(){
    return shooter.setAngle(ShooterConstants.shootConstants.SetpointConstants.LARGE_ANGLE);
  }

   public Command resetDefault(){
        return 
            shooter.setAngle(ShooterConstants.shootConstants.SetpointConstants.SMALL_ANGLE)
            .andThen(shooter.stopShooterMotorCmd());
    }

}