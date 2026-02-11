package frc.robot.subsystems;

import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.hardware.Pigeon2;

@ExtendWith(MockitoExtension.class)
public class DriveTest {
private Drive drive;

  @Mock private ModuleKraken fl, fr, rl, rr;
  @Mock private Pigeon2 gyro;
  @Mock private Vision vision;
  @Mock private PIDController rotPid, xPid, yPid;

  @BeforeEach
  void setUp() {
    // Swerve Odometry needs initial positions to not null-pointer
    SwerveModulePosition defaultPos = new SwerveModulePosition(0, new Rotation2d());
    when(fl.getSwerveModulePosition()).thenReturn(defaultPos);
    when(fr.getSwerveModulePosition()).thenReturn(defaultPos);
    when(rl.getSwerveModulePosition()).thenReturn(defaultPos);
    when(rr.getSwerveModulePosition()).thenReturn(defaultPos);
    when(gyro.getRotation2d()).thenReturn(new Rotation2d());

    // We use a 'spy' so we can override getShooterPose2d() without mocking the whole class
    drive = spy(new Drive(fl, fr, rl, rr, gyro, vision, rotPid, xPid, yPid));
  }

 @Test
void testRotationMath() {
    double robotX = 6;
    double robotY = 8;
    Rotation2d robotRotation = new Rotation2d(Math.PI/2); 
    Pose2d fakeShooterPose = new Pose2d(robotX, robotY, robotRotation);
    doReturn(fakeShooterPose).when(drive).getShooterPose2d();

    double targetX = 10;
    double targetY = 10;
    Translation2d visionTarget = new Translation2d(targetX, targetY);
    when(vision.getTargetTranslation(anyInt())).thenReturn(visionTarget);

    Rotation2d calculatedAngle = drive.unitTestRotation();

    double actualTargetX = targetX + 0.343; 
    double actualTargetY = targetY;

    double dx = actualTargetX - robotX; 
    double dy = actualTargetY - robotY; 

    double displacementAngleRad = Math.atan2(dy, dx);
    double expectedRadians = displacementAngleRad + Math.toRadians(90);

    expectedRadians = MathUtil.angleModulus(expectedRadians);
    double resultRadians = MathUtil.angleModulus(calculatedAngle.getRadians());
    
    assertEquals(expectedRadians, resultRadians, 0.001, 
        "tsk tsk tsk");
}
}
