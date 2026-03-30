package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoShooter;
import frc.robot.subsystems.Drive;

public final class Autos {

    private Autos() {}

    public static SendableChooser<Command> configureAuto(Drive drive, AutoShooter autoshooter) {

      // 2. Configure the AutoBuilder
      AutoBuilder.configure(
          drive::getPose2d,              // Robot pose supplier
          drive::resetPose,             // Method to reset odometry (will be called if your auto has a starting pose)
          drive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> drive.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants
          ),
          new RobotConfig(52.000, 4.477, 
            new ModuleConfig(0.051, 7.500, 1.200, DCMotor.getKrakenX60(1).withReduction(694), 1, 1), 
            0.736),
          () -> false,
          drive);

        NamedCommands.registerCommand("shoot", autoshooter.runAutonShooterMotorCmd().withTimeout(2.0));
        NamedCommands.registerCommand("index", autoshooter.runIndexerMotorCmd().withTimeout(3.0));
        
      SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
      return chooser;
    }
}
