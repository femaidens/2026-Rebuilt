// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    private PhotonCamera leftCam, rightCam;
    private final List<Pose3d> logVision = new ArrayList<>();

    private final Translation3d RIGHT_ROBOT_TO_CAM_TRANS;
    private final Rotation3d RIGHT_ROBOT_TO_CAM_ROT;
    private final Transform3d RIGHT_ROBOT_TO_CAM;

    private final Translation3d LEFT_ROBOT_TO_CAM_TRANS;
    private final Rotation3d LEFT_ROBOT_TO_CAM_ROT;
    private final Transform3d LEFT_ROBOT_TO_CAM;

    private final PhotonPoseEstimator rightEstimator;
    private final PhotonPoseEstimator leftEstimator;

    private final Drive drive;
   
    public Vision(){        
        leftCam = new PhotonCamera("2265-ironfish");
        rightCam = new PhotonCamera("2265-greenfish");

        RIGHT_ROBOT_TO_CAM_TRANS = new Translation3d(
            Units.inchesToMeters(11.248), 
            Units.inchesToMeters(-8.818), 
            Units.inchesToMeters(9));
        RIGHT_ROBOT_TO_CAM_ROT = new Rotation3d(
            0,
            0,
            0);
        RIGHT_ROBOT_TO_CAM = new Transform3d(
            RIGHT_ROBOT_TO_CAM_TRANS,
            RIGHT_ROBOT_TO_CAM_ROT
        );

        LEFT_ROBOT_TO_CAM_TRANS = new Translation3d(
            Units.inchesToMeters(11.248), 
            Units.inchesToMeters(8.818), 
            Units.inchesToMeters(9));
        LEFT_ROBOT_TO_CAM_ROT = new Rotation3d(
            0, 
            0, 
            0);
        LEFT_ROBOT_TO_CAM = new Transform3d(
            LEFT_ROBOT_TO_CAM_TRANS,
            LEFT_ROBOT_TO_CAM_ROT
        );

        rightEstimator = new PhotonPoseEstimator(TAG_LAYOUT, RIGHT_ROBOT_TO_CAM);
        leftEstimator = new PhotonPoseEstimator(TAG_LAYOUT, LEFT_ROBOT_TO_CAM); 

                drive = new Drive();

    }


 
    public Translation2d getTargetTranslation(int tagID){
        var tagPose = TAG_LAYOUT.getTagPose(tagID);
        return tagPose.get().toPose2d().getTranslation();
    }

    public List<EstimatedRobotPose> getVisionUpdates(){
        List<EstimatedRobotPose> results = new ArrayList<>();
        logVision.clear();
            for(var result:rightCam.getAllUnreadResults()){
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
        
             for(var result:leftCam.getAllUnreadResults()){
                Optional<EstimatedRobotPose> leftPose = leftEstimator.estimateCoprocMultiTagPose(result);
                if(leftPose.isPresent()) {
                    results.add(leftPose.get());
                    logVision.add(leftPose.get().estimatedPose);
                }
                else {
                    leftPose = leftEstimator.estimateLowestAmbiguityPose(result);
                        if(leftPose.isPresent()){
                            results.add(leftPose.get());
                            logVision.add(leftPose.get().estimatedPose);

                        }
                }
            }
            
    
        return results;
        
    }
}