package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
//import java.util.logging.Handler;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.epilogue.Logged;

@Logged
public class Vision  {
    private final AprilTagFieldLayout TAG_LAYOUT = 
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private PhotonCamera backSwerveCam, rightSwerveCam, batteryCam, shooterCam;
    private final List<Pose3d> logVision = new ArrayList<>();

    private final Translation3d RIGHT_ROBOT_TO_CAM_TRANS;
    private final Rotation3d RIGHT_ROBOT_TO_CAM_ROT;
    private final Transform3d RIGHT_ROBOT_TO_CAM;

    private final Translation3d BACK_ROBOT_TO_CAM_TRANS;
    private final Rotation3d BACK_ROBOT_TO_CAM_ROT;
    private final Transform3d BACK_ROBOT_TO_CAM;

    private final Translation3d BATTERY_ROBOT_TO_CAM_TRANS;
    private final Rotation3d BATTERY_ROBOT_TO_CAM_ROT;
    private final Transform3d BATTERY_ROBOT_TO_CAM;

    private final Translation3d SHOOTER_ROBOT_TO_CAM_TRANS;
    private final Rotation3d SHOOTER_ROBOT_TO_CAM_ROT;
    private final Transform3d SHOOTER_ROBOT_TO_CAM;

    private final PhotonPoseEstimator rightEstimator;
    private final PhotonPoseEstimator backEstimator;
    private final PhotonPoseEstimator batteryEstimator;
    private final PhotonPoseEstimator shooterEstimator;

    private final Drive drive;
   
    public Vision(){        
        backSwerveCam = new PhotonCamera("back swerve");
        rightSwerveCam = new PhotonCamera("right swerve");
        batteryCam = new PhotonCamera("battery cam"); // BOTTOM PORT ON PI, diagonal from BLUE
        shooterCam = new PhotonCamera("shooter cam");

        RIGHT_ROBOT_TO_CAM_TRANS = new Translation3d(
            Units.inchesToMeters(11.248), 
            Units.inchesToMeters(-8.818), 
            Units.inchesToMeters(9));
        RIGHT_ROBOT_TO_CAM_ROT = new Rotation3d(
            0,
            15*Math.PI/180,
            270*Math.PI/180);
        RIGHT_ROBOT_TO_CAM = new Transform3d(
            RIGHT_ROBOT_TO_CAM_TRANS,
            RIGHT_ROBOT_TO_CAM_ROT
        );

        BACK_ROBOT_TO_CAM_TRANS = new Translation3d(
            Units.inchesToMeters(11.248), // center to edge FORWARD
            Units.inchesToMeters(8.818), // center to edge SIDE
            Units.inchesToMeters(9)); // HEIGHT
        BACK_ROBOT_TO_CAM_ROT = new Rotation3d(
            0, 
            15*Math.PI/180, 
            213.806888*Math.PI/180);
        BACK_ROBOT_TO_CAM = new Transform3d(
            BACK_ROBOT_TO_CAM_TRANS,
            BACK_ROBOT_TO_CAM_ROT
        );

        BATTERY_ROBOT_TO_CAM_TRANS = new Translation3d(
            Units.inchesToMeters(-4.1372), 
            Units.inchesToMeters(13.54), 
            Units.inchesToMeters(9.1984));
        BATTERY_ROBOT_TO_CAM_ROT = new Rotation3d(
            0, 
            15*Math.PI/180, 
            90*Math.PI/180);
        BATTERY_ROBOT_TO_CAM = new Transform3d(
            BACK_ROBOT_TO_CAM_TRANS,
            BACK_ROBOT_TO_CAM_ROT
        );

        SHOOTER_ROBOT_TO_CAM_TRANS = new Translation3d(
            Units.inchesToMeters(-12.553), 
            Units.inchesToMeters(14.450), 
            Units.inchesToMeters(10.73)); // add radius of camera lens
        SHOOTER_ROBOT_TO_CAM_ROT = new Rotation3d(
            0, 
            15*Math.PI/180, 
            174.9165*Math.PI/180);
        SHOOTER_ROBOT_TO_CAM = new Transform3d(
            SHOOTER_ROBOT_TO_CAM_TRANS,
            SHOOTER_ROBOT_TO_CAM_ROT
        );

        rightEstimator = new PhotonPoseEstimator(TAG_LAYOUT, RIGHT_ROBOT_TO_CAM);
        backEstimator = new PhotonPoseEstimator(TAG_LAYOUT, BACK_ROBOT_TO_CAM); 
        batteryEstimator = new PhotonPoseEstimator(TAG_LAYOUT, BATTERY_ROBOT_TO_CAM);
        shooterEstimator = new PhotonPoseEstimator(TAG_LAYOUT, SHOOTER_ROBOT_TO_CAM);

        drive = new Drive();
    }


 
    public Translation2d getTargetTranslation(int tagID){
        var tagPose = TAG_LAYOUT.getTagPose(tagID);
        return tagPose.get().toPose2d().getTranslation();
    }

    public List<EstimatedRobotPose> getVisionUpdates(){
        List<EstimatedRobotPose> results = new ArrayList<>();
        logVision.clear();
            for(var result:rightSwerveCam.getAllUnreadResults()){
                Optional<EstimatedRobotPose> rightPose = rightEstimator.estimateCoprocMultiTagPose(result);
                if(rightPose.isPresent()) {
                    results.add(rightPose.get());
                    logVision.add(rightPose.get().estimatedPose);

                }
                else {
                    rightPose = rightEstimator.estimateLowestAmbiguityPose(result);
                        if(rightPose.isPresent()){
                            results.add(rightPose.get());
                            logVision.add(rightPose.get().estimatedPose);

                        }
                }
            }
        
             for(var result:backSwerveCam.getAllUnreadResults()){
                Optional<EstimatedRobotPose> leftPose = backEstimator.estimateCoprocMultiTagPose(result);
                if(leftPose.isPresent()) {
                    results.add(leftPose.get());
                    logVision.add(leftPose.get().estimatedPose);
                }
                else {
                    leftPose = backEstimator.estimateLowestAmbiguityPose(result);
                        if(leftPose.isPresent()){
                            results.add(leftPose.get());
                            logVision.add(leftPose.get().estimatedPose);

                        }
                }
            }

            for(var result:batteryCam.getAllUnreadResults()){
                Optional<EstimatedRobotPose> leftPose = backEstimator.estimateCoprocMultiTagPose(result);
                if(leftPose.isPresent()) {
                    results.add(leftPose.get());
                    logVision.add(leftPose.get().estimatedPose);
                }
                else {
                    leftPose = backEstimator.estimateLowestAmbiguityPose(result);
                        if(leftPose.isPresent()){
                            results.add(leftPose.get());
                            logVision.add(leftPose.get().estimatedPose);

                        }
                }
            }

            for(var result:shooterCam.getAllUnreadResults()){
                Optional<EstimatedRobotPose> leftPose = backEstimator.estimateCoprocMultiTagPose(result);
                if(leftPose.isPresent()) {
                    results.add(leftPose.get());
                    logVision.add(leftPose.get().estimatedPose);
                }
                else {
                    leftPose = backEstimator.estimateLowestAmbiguityPose(result);
                        if(leftPose.isPresent()){
                            results.add(leftPose.get());
                            logVision.add(leftPose.get().estimatedPose);

                        }
                }
            }
        return results;
    }
}
