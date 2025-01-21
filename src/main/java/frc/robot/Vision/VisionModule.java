package frc.robot.Vision;

import java.lang.annotation.Target;
import java.util.Optional;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightBetter;
import frc.robot.LimelightBetter.LEDMode;
import frc.robot.LimelightBetter.TargetData;

public class VisionModule {
  public LEDMode currentLedMode = LEDMode.ForceOff;
  public CameraMode currentCameraMode = CameraMode.VISION;
  public LinearFilter distanceFilter;
  public double pipelineLatency = 0;
  public double pipelineID = 0;
  public String name;
  public double ta;
  public boolean tv;
  public double tx;
  public double ty;
  public double fiducialID;
  public double tClass;
  public double boundingHorizontalPixels;
  public double boundingVerticalPixels;
  public VisionModuleConstants config;
  public double rawDistanceToTarget;
  public double distanceToTarget;
  LimelightBetter limelight;

  public double xOffset;

  public VisionModule(VisionModuleConstants cfg) {
    this.config = cfg;
    name = cfg.name;
    limelight = new LimelightBetter(name, true); 
    setLEDS(LEDMode.ForceOff);
    setCamMode(CameraMode.VISION);
    setPipeline(cfg.pipelineIndex);
    distanceFilter = LinearFilter.movingAverage(cfg.movingAverageNumTaps);
  }

  public void setOffset(double offset) {
    this.xOffset = offset;
  }

  public void setLEDS(LEDMode mode) {
    limelight.setLEDMode(mode);
  }

  public void setCamMode(CameraMode mode) {
    switch (mode) {
      case DRIVERCAM:
        limelight.setCameraMode(false);
        currentCameraMode = CameraMode.DRIVERCAM;
      case VISION:
        limelight.setCameraMode(true);
        currentCameraMode = CameraMode.VISION;
      case NONE:
        currentCameraMode = CameraMode.NONE;
    }
  }

  public void setPipeline(int index) {
    limelight.setPipeline(index);
  }

  public void periodic() {
    pipelineID = limelight.get("pipeline").getDouble();
    pipelineLatency = limelight.get("tl").getDouble();
    ta = limelight.get(TargetData.TA);
    tv = limelight.getTV();
    tx = limelight.get(TargetData.TX);
    ty = limelight.get(TargetData.TY);
    fiducialID = LimelightHelpers.getFiducialID(name);
    boundingHorizontalPixels = limelight.get(TargetData.thor);
    boundingVerticalPixels = limelight.get(TargetData.tvert);
    tClass = LimelightHelpers.getNeuralClassID(name);
    rawDistanceToTarget = getDistToTarget();
    distanceToTarget = distanceFilter.calculate(rawDistanceToTarget);
  }

  public Pose2d getBotPose() {
    double[] botPose;
    if (getAlliance() == "Red") {
      botPose = LimelightHelpers.getBotPose_wpiRed(name);

    } else {
      botPose = LimelightHelpers.getBotPose_wpiBlue(name);
    }
    double xPose = botPose[0];
    double yPose = botPose[1];
    double yawPose = botPose[5];
    return new Pose2d(new Translation2d(xPose, yPose), new Rotation2d(yawPose));
  }

  public double getDistToTarget() {
    if (config.moduleType == VisionModuleType.apriltag) {
      return LimelightHelpers.getTargetPose_RobotSpace(name)[3];
    }
    return distanceFromCameraPercentage(boundingHorizontalPixels);
  }

  public String getAlliance() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        return "Red";
      }
      if (ally.get() == Alliance.Blue) {
        return "Blue";
      }
    }
    return "";
  }

  public double pixlesToPercent(double pixels) {
    return pixels / config.horPixles;
  }

  /**
   * 
   * @param widthPercent [0,1], percentage of the vertical width of the image that
   *                     the note is taking up
   * @return distance in meters
   */
  public double distanceFromCameraPercentage(double widthPercent) {

    if (limelight.getTV()) {
      widthPercent = pixlesToPercent(widthPercent);
      double hypotDist = ((180 * Units.inchesToMeters(14)) / (63.3 * Math.PI)) * (1 / widthPercent);
      double intakeDist = Math
          .sqrt((hypotDist * hypotDist) - (config.limelightMountHeight * config.limelightMountHeight)); // distance to
                                                                                                        // intake
      return intakeDist;
    } else {
      return 0;
    }
  }

  public Pose2d getTargetPose() {
    return new Pose2d(getDistToTarget(), getTargetY(), Rotation2d.fromDegrees(tx));
  }

  public double getTargetY() {
    return getDistToTarget() * Math.cos(Math.toRadians(90 - tx));
  }
}
