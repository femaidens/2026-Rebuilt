// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.Shooting;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot(Drive drivetrain, Intake intake, Hopper hopper, Shooter shooter, Shooting shooting) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroHeading()),
      new RunCommand(() -> drivetrain.drive(() -> 0.0, () -> 1.0, () -> 0.0), drivetrain)
        .withTimeout(5),
      hopper.runIndexer()
        .withTimeout(2),
      shooting.middleAngleCmd()
        .withTimeout(3)
    );
  }
}
