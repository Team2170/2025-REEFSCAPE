// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.util.DSUtil;
import frc.robot.util.RotationUtil;
import frc.robot.util.VisionObservation.LLTYPE;

public class VisionIOLimelight implements VisionIO {
  /** Creates a new VisionIOLimelight. */
  LEDMode currentLedMode = LEDMode.FORCEOFF;

  CamMode currentCamMode = CamMode.VISION;
  public final limelightConstants constants;
  private final String name;
  private final LLTYPE limelightType;

  public VisionIOLimelight(limelightConstants limelightConstants) {
    constants = limelightConstants;
    name = constants.name;
    limelightType = constants.limelightType;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.ledMode = currentLedMode;
    // inputs.camMode = currentCamMode;
    inputs.pipelineID = LimelightHelpers.getCurrentPipelineIndex(name);
    inputs.pipelineLatency = LimelightHelpers.getLatency_Pipeline(name);
    inputs.ta = LimelightHelpers.getTA(name);
    inputs.tv = LimelightHelpers.getTV(name);
    inputs.tx = LimelightHelpers.getTX(name); // TODO add limelight disconnect alert
    inputs.ty = LimelightHelpers.getTY(name);
    inputs.fiducialID = LimelightHelpers.getFiducialID(name);
    String llClass = LimelightHelpers.getNeuralClassID(name);
    inputs.tClass = llClass.isEmpty() ? 0 : Double.parseDouble(llClass);
    inputs.name = name;
    inputs.botPoseMG2 = getBotPoseMg2(name);
    inputs.tagCount = getTagCount(name);
    inputs.avgTagDist = getAvgDist(name);
    inputs.botPose3d = getBotPose3d(name);
    inputs.timestamp = getTimestamp(name);
    inputs.limelightType = limelightType;
  }

  public Pose2d getBotPoseMg2(String camName) {
    PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);
    if (poseEst == null) {
      return new Pose2d();
    }
    return poseEst.pose;
  }

  public Pose3d getBotPose3d(String camName) {
    Pose3d poseEst = LimelightHelpers.getBotPose3d_wpiBlue(camName);
    if (poseEst == null) {
      return new Pose3d();
    }
    return poseEst;
  }

  public int getTagCount(String camName) {
    PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);
    if (poseEst == null) {
      return 0;
    }
    return poseEst.tagCount;
  }

  public double getAvgDist(String camName) {
    PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);
    if (poseEst == null) {
      return 0.0;
    }
    return poseEst.avgTagDist;
  }

  public double getTimestamp(String camName) {
    PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);
    if (poseEst == null) {
      return 0.0;
    }
    return poseEst.timestampSeconds;
  }

  @Override
  public void setLEDS(LEDMode mode) {
    switch (mode) {
      case FORCEBLINK:
        LimelightHelpers.setLEDMode_ForceBlink(name);
        currentLedMode = LEDMode.FORCEBLINK;
        break;
      case FORCEOFF:
        LimelightHelpers.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
      case FORCEON:
        LimelightHelpers.setLEDMode_ForceOn(name);
        currentLedMode = LEDMode.FORCEON;
      case PIPELINECONTROL:
        LimelightHelpers.setLEDMode_PipelineControl(name);
        currentLedMode = LEDMode.PIPELINECONTROL;
      default:
        LimelightHelpers.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
        break;
    }
  }

  @Override
  public void setPipeline(String limelight, int index) {
    LimelightHelpers.setPipelineIndex(limelight, index);
  }

  public void setRobotOrientationMG2(Rotation3d gyro, Rotation3d rate) {
    gyro = DSUtil.isBlue() ? gyro : gyro.rotateBy(new Rotation3d(new Rotation2d(Math.PI)));
    Rotation3d gyroval = RotationUtil.wrapRot3d(gyro);
    Rotation3d rateval = RotationUtil.wrapRot3d(rate);

    LimelightHelpers.SetRobotOrientation(
        name,
        gyroval.getZ(),
        rateval.getZ(),
        gyroval.getY(),
        rateval.getY(),
        gyroval.getX(),
        rateval.getX());
  }

  @Override
  public void setPermittedTags(int[] tags) {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, tags);
  }

  @Override
  public void setPriorityID(int tagID) {
    NetworkTableInstance.getDefault().getTable(name).getEntry("priorityid").setDouble(tagID);
  }
}
