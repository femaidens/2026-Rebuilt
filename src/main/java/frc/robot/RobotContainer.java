// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.KaileyLeds;
import frc.robot.subsystems.KaseyLeds;
import frc.robot.subsystems.LydiaLeds;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

@Logged

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driveJoy = new CommandXboxController(OperatorConstants.DRIVER_PORT);
    private final CommandXboxController operJoy = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

    private Intake intake = new Intake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

        // operJoy.leftTrigger().onTrue(lydia.setDefault());
        
  //       /* 
  //       operJoy.a().onTrue(lydia.setBumperBlueSolid());
  //       operJoy.x().onTrue(lydia.setBumperRedSolid());
  //       operJoy.b().onTrue(lydia.setPurpleSolid());
  //       operJoy.y().onTrue(lydia.setGreenSolid());
  //       operJoy.leftBumper().onTrue(lydia.setFemaidensSplit());
  //       operJoy.rightBumper().onTrue(lydia.setFirstRedSolid());
  //       operJoy.rightTrigger().onTrue(lydia.setFemaidensPan());
  //       operJoy.start().onTrue(lydia.setFemaidensStaticGrad());
  //       operJoy.back().onTrue(lydia.setFemaidensScrollGrad());
  //       */


  //       // operJoy.rightTrigger().onTrue(kailey.clear());

  //       // operJoy.a().onTrue(kailey.solidPurple());
  //       // operJoy.x().onTrue(kailey.solidGreen());
  //       // operJoy.y().onTrue(kailey.pulseEffect());
  //       // operJoy.b().onTrue(kailey.sparkleEffect());
  //       // operJoy.leftBumper().onTrue(kailey.progressMask());
  //       // operJoy.rightBumper().onTrue(kailey.offsetGradient());
  //       // operJoy.leftTrigger().onTrue(kailey.greenGradient());
  //       // operJoy.start().onTrue(kailey.purpleGradient());



  //        operJoy.leftTrigger().onTrue(kasey.setDefault());

  //       operJoy.a().onTrue(kasey.setPurpleCommand());
  //        operJoy.x().onTrue(kasey.setGreenCommand());
  //        operJoy.y().onTrue(kasey.setRedCommand());
  //        operJoy.b().onTrue(kasey.setBlueCommand());
  //        operJoy.leftBumper().onTrue(kasey.breatheEffect());
  //        operJoy.rightBumper().onTrue(kasey.progressMaskEffect());
  //        operJoy.leftTrigger().onTrue(kasey.setPinkCommand());
  //        operJoy.start().onTrue(kasey.progressMaskEffectGreenPurple());

  //   // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
  //   // cancelling on release.
    
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    operJoy.a().whileTrue(intake.setAngleDownCmd());
    operJoy.b().whileTrue(intake.setAngleUpCmd());
    operJoy.x().onTrue(intake.setAngleUpDownCmd());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   //return Autos.exampleAuto(m_exampleSubsystem);
  // }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // // public Command getAutonomousCommand() {
  // //   // An example command will be run in autonomous
  // //   // return Autos.exampleAuto(m_exampleSubsystem);
  // //   //return Autos.exampleAuto(m_exampleSubsystem);
  // // }
}
