// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Limelight;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Limelight.VisionObservation.LLTYPE;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private SwerveDrive swerve;
  public boolean apriltagPipeline;
  private double xyStdDev;
  private double thetaStdDev;
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  public Vision(SwerveDrive swerve, VisionIO io) {
    this.io = io;
    // this.yaw = yaw;
    this.swerve = swerve;

    io.setLEDS(LEDMode.FORCEOFF);
  }

  public void setLEDS(boolean on) {
    io.setLEDS(on ? LEDMode.FORCEBLINK : LEDMode.PIPELINECONTROL);
  }

  public double getTClass() {
    return inputs.tClass;
  }

  public boolean getTV() {
    return inputs.tv;
  }

  public double getID() {
    return inputs.fiducialID;
  }

  public void setPipeline(int id) {
    io.setPipeline(inputs.name, id);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Limelight" + inputs.name, inputs);
    LimelightHelpers.SetFiducialIDFiltersOverride(
        inputs.name, AprilTagVisionConstants.limelightConstants.validTags);

    apriltagPipeline = inputs.pipelineID == 0;

    if (inputs.limelightType != LLTYPE.LL4 && DriverStation.isDSAttached()) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red) {

        Rotation3d yaw3d = new Rotation3d(swerve.getGyroYaw());
        Rotation3d gyro = yaw3d.rotateBy(new Rotation3d(0, 0, Math.PI));
        io.setRobotOrientationMG2(gyro, swerve.getBaseGyro().getRotationRate());

      } else {
        Rotation3d yaw3d = new Rotation3d(swerve.getGyroYaw());
        io.setRobotOrientationMG2(yaw3d, swerve.getBaseGyro().getRotationRate());
      }
    }

    if (inputs.tagCount < 2) {
      xyStdDev = AprilTagVisionConstants.limelightConstants.xySingleTagStdDev;
      thetaStdDev = AprilTagVisionConstants.limelightConstants.thetaSingleTagStdDev;
    } else {
      xyStdDev = AprilTagVisionConstants.limelightConstants.xyMultiTagStdDev;
      thetaStdDev = AprilTagVisionConstants.limelightConstants.thetaMultiTagStdDev;
    }

    if (inputs.limelightType == LLTYPE.LL4) {
      if (getPoseValidMG2(swerve.getGyroYaw())) {
        updatePose();
      }
    } else {
      if (swerve.getBaseGyro().getRotationRate().getZ() <= Units.degreesToRadians(720)) {
        updatePose();
      }
    }

    if (inputs.name != "sim") {
      PoseEstimate estimate =  LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(inputs.name);
      if( estimate != null){
        LimelightHelpers.RawFiducial[] rawTrackedTags = estimate.rawFiducials;
        List<Integer> trackedTagID = new ArrayList<Integer>();
  
        for (int i = 0; i < rawTrackedTags.length; i++) {
          trackedTagID.add(rawTrackedTags[i].id);
        }
  
        Pose2d[] trackedTagPoses = new Pose2d[rawTrackedTags.length];
        for (int i = 0; i < trackedTagID.size(); i++) {
          trackedTagPoses[i] = aprilTagFieldLayout.getTagPose(trackedTagID.get(i)).get().toPose2d();
        }
        
        Logger.recordOutput("limelight" + inputs.name + "/visionTargets", trackedTagPoses);
      }
    }
  }

  public Pose2d getBotPoseMG2() {
    return inputs.botPoseMG2;
  }

  public void resetGyroLL4(SwerveDrive drive) {
    if (DriverStation.isDSAttached()) {
      LimelightHelpers.SetIMUMode(inputs.name, 1);
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red) {
        Rotation3d rr = new Rotation3d();
        Rotation3d yaw3d = new Rotation3d(drive.getGyroYaw());
        Rotation3d gyro = yaw3d.rotateBy(new Rotation3d(0, 0, Math.PI));
        io.setRobotOrientationMG2(gyro, rr);

      } else {
        Rotation3d rr = new Rotation3d();
        Rotation3d yaw3d = new Rotation3d(drive.getGyroYaw());
        io.setRobotOrientationMG2(yaw3d, rr);
      }

      LimelightHelpers.SetIMUMode(inputs.name, 2);
    }
  }

  /**
   * TODO fix mount angle, this assumes the limelight is mounted at the back of
   * the robot
   *
   * @param yaw current yaw of the robot
   * @return the angle of the focused apriltag relative to the robot
   */
  public Rotation2d txToYaw(Rotation2d yaw) {
    Rotation2d output = yaw.minus(getTX());
    Logger.recordOutput("Vision/TXtoYaw", output);
    return output;
  }

  /**
   * @return the translation between the primary in view apriltag and the camera
   */
  public Translation2d targetPoseCameraSpace() {
    Logger.recordOutput(
        "limelight/singletagdist", LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[2]);

    return new Translation2d(
        LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[0],
        LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[2]);
  }

  /**
   * @param tags anything NOT in here will be thrownOut
   */
  public void setPermittedTags(int[] tags) {
    io.setPermittedTags(tags);
  }

  /** */
  public boolean getPoseValidMG2(Rotation2d gyro) {

    // get raw data from limelight pose estimator
    Pose2d botpose = inputs.botPoseMG2;
    double diff = 0;

    double gyroval = gyro.getDegrees();
    gyroval = gyroval % (360);

    double x = botpose.getX();
    double y = botpose.getY();

    double tagDist = inputs.avgTagDist;

    // debugging purposes only
    Logger.recordOutput("LLDebug/" + inputs.name + " avgTagDist", tagDist);
    Logger.recordOutput("LLDebug/" + inputs.name + " tagCount", inputs.tagCount);
    Logger.recordOutput("LLDebug/" + inputs.name + " x val", x);
    Logger.recordOutput("LLDebug/" + inputs.name + " y val", y);
    Logger.recordOutput("LLDebug/" + inputs.name + " rdiff", diff);

    // this determines if the raw data from the limelight is valid
    // sometimes the limelight will give really bad data, so we want to throw this
    // out
    // and not use it in our pose estimation.
    // to check for this, we check to see if the rotation from the pose matches
    // the rotation that the gyro is reporting
    // we then check if the pose is actually within the bounds of the field
    // if all these requirements are met, then we can trust the measurement
    // otherwise we ignore it.

    if ((diff < AprilTagVisionConstants.limelightConstants.rotationTolerance)
        && (tagDist < AprilTagVisionConstants.limelightConstants.throwoutDist)
        && (botpose.getTranslation().getX() > 0)
        && (botpose.getTranslation().getX() < AprilTagVisionFieldConstants.fieldLength)
        && (botpose.getTranslation().getY() > 0)
        && (botpose.getTranslation().getY() < AprilTagVisionFieldConstants.fieldWidth)) {

      return true;
    } else {
      return false;
    }
  }

  public Pose3d getBotPose3d() {
    Pose3d pose = inputs.botPose3d;
    Logger.recordOutput("Limelight" + inputs.name + "/Pose3d", pose);
    return pose;
  }

  public double getPoseTimestampMG2() {
    return inputs.timestamp;
  }

  public String getLimelightName() {
    return inputs.name;
  }

  // angle target is from the center of the limelights crosshair
  public Rotation2d getTX() {
    return Rotation2d.fromDegrees(inputs.tx);
  }

  public double getTA() {
    return inputs.ta;
  }

  public void setPriorityID(int tagID) {
    io.setPriorityID(tagID);
  }

  public double tagCount() {
    return inputs.tagCount;
  }

  public void updatePose() {
    VisionObservation visionObservation = new VisionObservation(
        getBotPoseMG2(),
        getPoseTimestampMG2(),
        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
    swerve.swerveDrivePoseEstimator.swerveDrivePoseEstimator.addVisionMeasurement(
        visionObservation.getPose(),
        visionObservation.getTimestamp(),
        visionObservation.getStdDev());
  }
}
