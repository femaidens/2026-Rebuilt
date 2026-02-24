// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AutoShooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.ShooterConstants;
import frc.robot.ShooterConstants.shootConstants.PIDConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public final class Autos {

  private final Drive drivetrain;
  private final Shooter shooter;
  private final Shooting shooting;
  private RobotContainer config;
  private SendableChooser<Command> autonChooser;


  public Autos(Drive drive, Shooter shooter, Shooting shooting) {

    autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Choose Auto: ", autonChooser);
   
    this.drivetrain = drive;
    this.shooter = shooter;
    this.shooting = shooting;

  }

  public boolean isRedAlliance(){
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
  }

  public SendableChooser<Command> configure() {
    try{
      config = config.fromGUISettings();
    }catch(
    Exception e)
    {
    // Handle exception as needed
    e.printStackTrace();
    }

    // AutoBuilder.configure(
    //   drivetrain::getPose, 
    //   drivetrain::resetOdometry,
    //   drivetrain::getCurrentChassisSpeeds,
    //   (s, feedforwards) -> drivetrain.setChassisSpeeds(s),
    //   new PPHolonomicDriveController(
    //     new PIDConstants(), 
    //     new PIDConstants()),
    //   config,
    //   () -> isRedAlliance(),
    //   drivetrain);

    //   NamedCommands.registerCommand("shoot max level", largeAngleCmd());


    //   autonChooser = AutoBuilder.buildAutoChooser("Blue Left to Reef Front");

      autonChooser.addOption("No auto", Commands.none());

      return autonChooser;
  }
}