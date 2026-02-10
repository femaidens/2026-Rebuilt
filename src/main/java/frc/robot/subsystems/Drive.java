// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.SignalLogger;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;
import edu.wpi.first.epilogue.Logged;

@Logged
public class Drive extends SubsystemBase {

  private final Vision vision;

  private final PIDController rotPidController;
  private final PIDController xPidController;
  private final PIDController yPidController;

  private final SwerveDrivePoseEstimator swerveEstimator;

  private final ModuleKraken frontLeft;
  private final ModuleKraken frontRight;
  private final ModuleKraken rearLeft;
  private final ModuleKraken rearRight;
  // private double xSpeed;
  // private double ySpeed;
  //private double rotSpeed;

  private DoubleSupplier xSpeed;
  private DoubleSupplier ySpeed;
  private DoubleSupplier rotSpeed;

  private final List<ModuleKraken> modules;

  // private final AHRS gyro;

  private final Pigeon2 gyro;

  public final SwerveDriveOdometry odometry;

  private final SysIdRoutine driveRoutine;

  private ChassisSpeeds speeds = new ChassisSpeeds();

  /** Creates a new Drive. */
  public Drive() {
    vision = new Vision();

    rotPidController = new PIDController(DriveConstants.Translation.rotPID.P, DriveConstants.Translation.rotPID.I,
        DriveConstants.Translation.rotPID.D);
    xPidController = new PIDController(DriveConstants.Translation.xPID.P, DriveConstants.Translation.xPID.I,
        DriveConstants.Translation.xPID.D);
    yPidController = new PIDController(DriveConstants.Translation.yPID.P, DriveConstants.Translation.yPID.I,
        DriveConstants.Translation.yPID.D);

    rotPidController.enableContinuousInput(-180, 180);
    xPidController.enableContinuousInput(-180, 180);
    yPidController.enableContinuousInput(-180, 180);

    rotPidController.setTolerance(3);
    xPidController.setTolerance(0.05);
    yPidController.setTolerance(0.05);

    // frontLeft = new ModuleSpark(DrivetrainPorts.FRONT_LEFT_DRIVE,
    // DrivetrainPorts.FRONT_LEFT_TURN, Translation.FRONT_LEFT_ANGOFFSET);
    // frontRight = new ModuleSpark(DrivetrainPorts.FRONT_RIGHT_DRIVE,
    // DrivetrainPorts.FRONT_RIGHT_TURN, Translation.FRONT_RIGHT_ANGOFFSET);
    // rearLeft = new ModuleSpark(DrivetrainPorts.REAR_LEFT_DRIVE,
    // DrivetrainPorts.REAR_LEFT_TURN, Translation.REAR_LEFT_ANGOFFSET);
    // rearRight = new ModuleSpark(DrivetrainPorts.REAR_RIGHT_DRIVE,
    // DrivetrainPorts.REAR_RIGHT_TURN, Translation.REAR_RIGHT_ANGOFFSET);
    frontLeft = new ModuleKraken(DrivetrainPorts.FRONT_LEFT_DRIVE, DrivetrainPorts.FRONT_LEFT_TURN,
        DrivetrainPorts.FRONT_LEFT_CANCODER, Translation.FRONT_LEFT_MAG_OFFSET, Translation.FRONT_LEFT_ANGOFFSET,
        false);
    frontRight = new ModuleKraken(DrivetrainPorts.FRONT_RIGHT_DRIVE, DrivetrainPorts.FRONT_RIGHT_TURN,
        DrivetrainPorts.FRONT_RIGHT_CANCODER, Translation.FRONT_RIGHT_MAG_OFFSET, Translation.FRONT_RIGHT_ANGOFFSET,
        false);
    rearLeft = new ModuleKraken(DrivetrainPorts.REAR_LEFT_DRIVE, DrivetrainPorts.REAR_LEFT_TURN,
        DrivetrainPorts.REAR_LEFT_CANCODER, Translation.REAR_LEFT_MAG_OFFSET, Translation.REAR_LEFT_ANGOFFSET, false);
    rearRight = new ModuleKraken(DrivetrainPorts.REAR_RIGHT_DRIVE, DrivetrainPorts.REAR_RIGHT_TURN,
        DrivetrainPorts.REAR_RIGHT_CANCODER, Translation.REAR_RIGHT_MAG_OFFSET, Translation.REAR_RIGHT_ANGOFFSET,
        false);

    modules = List.of(frontLeft, frontRight, rearLeft, rearRight);

    // totally not sure, would need to check

    // gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro = new Pigeon2(DrivetrainPorts.GYRO_ID, Translation.CAN_BUS);
    odometry = new SwerveDriveOdometry(
        Drivetrain.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(),
            rearRight.getSwerveModulePosition()
        });
    zeroHeading();

    swerveEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.Drivetrain.kDriveKinematics,
        gyro.getRotation2d(),
        getSwerveModulePosition(),
        new Pose2d(),
        DriveConstants.Drivetrain.STATE_STD_DEV,
        DriveConstants.Drivetrain.VISION_STD_DEV);

    driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null,
            // Volts.of(2).per(Seconds.of(1)),
            // Volts.of(9),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> modules.forEach(m -> m.setDriveVoltage(volts.in(Units.Volts))),
            null,
            this));
  }

  public Pose2d getPose2d() {
    return swerveEstimator.getEstimatedPosition();
  }

  public Pose2d getShooterPose2d(){
    Transform2d shooterOffset = new Transform2d(-0.254,0.2240026, new Rotation2d(-Math.PI/2));

    return swerveEstimator.getEstimatedPosition().plus(shooterOffset);
  }

  public void alignRotation(){
    var alliance = DriverStation.getAlliance();
    Pose2d shooterPose = this.getShooterPose2d();
    Translation2d targetLocation;
    Translation2d difference = new Translation2d(.343, 0);

     if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      targetLocation = vision.getTargetTranslation(10).minus(difference);
    } else {
      targetLocation = vision.getTargetTranslation(26).plus(difference);
    }

    Translation2d displacement = targetLocation.minus(shooterPose.getTranslation());

    Rotation2d targetAngle = displacement.getAngle().plus(Rotation2d.kCCW_90deg);

    double rotOutput = rotPidController.calculate(shooterPose.getRotation().getRadians(), targetAngle.getRadians());

    this.drive(() -> 0, () -> 0, () -> rotOutput);
  }



  public void driveToPose(double x, double y, double r) {
    Pose2d currentPose = this.getPose2d();
   double xVel = xPidController.calculate(currentPose.getX(), x);
   double yVel = yPidController.calculate(currentPose.getY(), y);
   double rad = r * (Math.PI/180);
   double rotVel = rotPidController.calculate(
        currentPose.getRotation().getRadians(),
        rad // double check where climb is
    );
    xVel = MathUtil.clamp(xVel, -2.0, 2.0);
    yVel = MathUtil.clamp(yVel, -2.0, 2.0);
    rotVel = MathUtil.clamp(rotVel, -Math.PI, Math.PI);

    this.driveRaw(xVel, yVel, rotVel);

  }

  public Command driveToPoseCommand(double x, double y, double r) {
    return this.run(() -> driveToPose(x, y, r));
  }

  public double distanceFromTarget() {
    Pose2d currentPose = this.getPose2d();
    var alliance = DriverStation.getAlliance();
    Translation2d difference = new Translation2d(.343, 0);
    Translation2d targetLocation;

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      targetLocation = vision.getTargetTranslation(10).minus(difference);
    } else {
      targetLocation = vision.getTargetTranslation(26).plus(difference);
    }

    return currentPose.getTranslation().getDistance(targetLocation);

  }

  

  // public void alignRotation(DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
  //   Pose2d currentPose = this.getPose2d();
  //   var alliance = DriverStation.getAlliance();
  //   Translation2d targetLocation;
  //   Translation2d difference = new Translation2d(.343, 0);
  //   double angularOffset;

  //   if (alliance.isPresent() && alliance.get() == Alliance.Red) {
  //     targetLocation = vision.getTargetTranslation(10).minus(difference);
  //     angularOffset = Math.atan2(0.259715, currentPose.getTranslation().getDistance(targetLocation)+259715) * (180 / Math.PI);
  //   } else {
  //     targetLocation = vision.getTargetTranslation(26).plus(difference);
  //     angularOffset = Math.atan2(0.259715, currentPose.getTranslation().getDistance(targetLocation)+.259715) * (180 / Math.PI);
  //   }

  //   Rotation2d targetAngle = targetLocation.minus(currentPose.getTranslation()).getAngle();

  //   double rotOutput = rotPidController.calculate(currentPose.getRotation().getDegrees(),
  //       targetAngle.getDegrees() + 90 + angularOffset);
  //   // for rotOutput, +90 bc shooter is 90 deg away from front of robot and
  //   // +angularOffset cuz shooter isn't centered

  //   double clampedrotOutput = MathUtil.clamp(rotOutput, -0.5, 0.5);

  //   this.drive(xSpeed, ySpeed, () -> clampedrotOutput);

  // }

  public void driveRaw(double xVel, double yVel, double rotVel) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, gyro.getRotation2d());
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  // consider changing to profiledpid control
  /**
   * drivin
   * 
   * @param xSpeed   x direction (front and back)
   * @param ySpeed   y direction (right is positive, left is negative)
   * @param rotSpeed
   * @return
   */
  public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    double xVel = xSpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double yVel = ySpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double rotVel = rotSpeed.getAsDouble() * Drivetrain.MAX_ROT_SPEED * Drivetrain.SPEED_FACTOR;

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, gyro.getRotation2d());
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);

    // return this.run(
    // () -> {
    // for(int i = 0; i < modules.size(); i++){
    // modules.get(i).setDesiredState(moduleStates[i]);
    // }
    // }
    // );
    setModuleStates(moduleStates);
  }

  public void driveRobotRelative(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    double xVel = xSpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double yVel = ySpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double rotVel = rotSpeed.getAsDouble() * Drivetrain.MAX_ROT_SPEED * Drivetrain.SPEED_FACTOR;

    speeds = new ChassisSpeeds(xVel, yVel, rotVel);
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);

    // return this.run(
    // () -> {
    // for(int i = 0; i < modules.size(); i++){
    // modules.get(i).setDesiredState(moduleStates[i]);
    // }
    // }
    // );
    setModuleStates(moduleStates);
  }

  /**
   * sets the swerve ModuleStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Drivetrain.MAX_SPEED);
    frontLeft.setDesiredStateNoPID(desiredStates[0]); // frontLeft.setDesiredStateNoPID(desiredStates[1]);
    frontRight.setDesiredStateNoPID(desiredStates[1]); // frontRight.setDesiredStateNoPID(desiredStates[0]);
    rearLeft.setDesiredStateNoPID(desiredStates[2]); // rearLeft.setDesiredStateNoPID(desiredStates[3]);
    rearRight.setDesiredStateNoPID(desiredStates[3]); // rearRight.setDesiredStateNoPID(desiredStates[2]);
  }

  public void setChassisSpeeds(ChassisSpeeds speedd) {
    // ChassisSpeeds x = ChassisSpeeds.fromFieldRelativeSpeeds(speedd, getAngle());
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speedd);
    setModuleStates(moduleStates);
  }

  public ChassisSpeeds getDesiredChassisSpeeds() {
    return speeds;
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    ChassisSpeeds spede = DriveConstants.Drivetrain.kDriveKinematics.toChassisSpeeds(
        getSwerveModuleStates()[0], getSwerveModuleStates()[1], getSwerveModuleStates()[2], getSwerveModuleStates()[3]);
    return spede;
  }

  public SwerveModulePosition[] getSwerveModulePosition() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] { frontLeft.getSwerveModulePosition(),
        frontRight.getSwerveModulePosition(), rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition() };
    return swerveModulePositions;
  }

  /**
   * x formation with wheels to prevent movement
   */
  public Command setXCmd() {
    return this.run(
        () -> {
          frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
          frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
          rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
          rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
        });
  }

  /**
   * Sets wheels straight for sysid
   */
  public Command setStraightCmd() {
    return this.run(
        () -> {
          frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
          frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
          rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
          // only rear right is acting up, consider changing it to 180 degrees
          rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        });
  }

  public Command driveStraightCmd() {
    return this.run(
        () -> {
          frontLeft.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
          frontRight.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
          rearLeft.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
          rearRight.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
        });
  }

  /**
   * @return currently-estimated pose of robot
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return modules.stream().map(m -> m.getState()).toArray(SwerveModuleState[]::new);
  }

  public SwerveModuleState[] getDesiredSwerveModuleStates() {
    return modules.stream().map(m -> m.getDesiredState()).toArray(SwerveModuleState[]::new);
  }

  public double[] getVoltage() {
    double[] voltages = { frontLeft.getVoltage(), frontRight.getVoltage(), rearLeft.getVoltage(),
        rearRight.getVoltage() };
    return voltages;
  }

  public double[] getAbsolutes() {
    double[] absolutes = { frontLeft.getAbsolute(), frontRight.getAbsolute(), rearLeft.getAbsolute(),
        rearRight.getAbsolute() };
    return absolutes;
  }

  /**
   * resets the odometry to the specified pose
   */
  public void resetOdometry(Pose2d pose) {
    // odometry.resetPosition(
    // Rotation2d.fromRadians(gyro.getYaw()),
    // new SwerveModulePosition[]{
    // frontLeft.getSwerveModulePosition(),
    // frontRight.getSwerveModulePosition(),
    // rearLeft.getSwerveModulePosition(),
    // rearRight.getSwerveModulePosition()}, pose);
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(),
            rearRight.getSwerveModulePosition() },
        pose);
  }

  /**
   * Gets the angle of the gyro in radians (ideally)
   * 
   * @return in radians
   */
  public double getAngle() {
    // return -1 * gyro.getAngle();
    return gyro.getYaw().getValueAsDouble();
  }

  /**
   * In case we'll ever need it
   * 
   * @return new yaw angle in radians (ideally)
   */
  // public double setYawOffset() {
  // gyro.setAngleAdjustment(-Math.PI / 2); // need to double check!
  // return gyro.getYaw();
  // }
  // public double getYawOffset(){
  // gyro.setYaw(0);
  // return gyro.getRotation2d().getRadians();
  // }
  /**
   * Zero the gyro heading
   */
  public void zeroHeading() {
    gyro.reset();
  }

  public Command resetGyro() {
    return this.runOnce(
        () -> zeroHeading());
  }

  // public Pose2d getOdometry(){
  // Pose2d odometry = swerveEstimator.update(gyro.getRotation2d(),
  // getSwerveModulePosition());

  // return odometry;
  // }

  /* SYSID CMDS */
  public Command driveQuasistatic(SysIdRoutine.Direction direction) {
    System.out.println("RUNNING");
    System.out.println("RUNNING");
    return driveRoutine.quasistatic(direction);
  }

  public Command driveDynamic(SysIdRoutine.Direction direction) {
    return driveRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if gyro is inverted, getRotation2d() --- getAngle() can be negated
    swerveEstimator.update(gyro.getRotation2d(), getSwerveModulePosition());

    List<EstimatedRobotPose> visionUpdates = vision.getVisionUpdates();
    for (EstimatedRobotPose update : visionUpdates) {
      swerveEstimator.addVisionMeasurement(
          update.estimatedPose.toPose2d(),
          update.timestampSeconds);
    }
    // SmartDashboard.getNumber("Angle", getAngle());
    SmartDashboard.putNumber("Gyro Angle", getAngle());
    SmartDashboard.updateValues();
    SmartDashboard.putNumber("angle", getAngle());
  }
}