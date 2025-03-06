// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Subsystems.Limelight.VisionObservation;
import frc.robot.Subsystems.Limelight.VisionObservation.LLTYPE;
import frc.robot.Subsystems.Limelight.limelightConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class AutoAlignmentConstants {
    public static final double DEADBAND = 0.1;
    public static final double ANGLE_KP = .5;
    public static final double ANGLE_KD = 0;
    public static final double DRIVE_KPY = 1;
    public static final double DRIVE_KDY = 0;
    public static final double DRIVE_KPX = 1;
    public static final double DRIVE_KDX = 0;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final Distance ALIGN_DISTANCE = Meters.of(.4);
    public static final double DRIVE_BASE_RADIUS = Math.max(
        Math.max(
            Math.hypot(
                -10.25, 10.25),
            Math.hypot(
                10.25, 10.25)),
        Math.max(
            Math.hypot(-10.25, -10.25),
            Math.hypot(
                10.25, -10.25)));
  }

  public static final class Limelight2GConstants {
    public static final double verticalFOV = 49.7; // degrees obviously
    public static final double horizontalFOV = 63.3;
    public static final int horPixles = 1280;
  }

  public static final class Limelight3Constants {
    public static final double verticalFOV = 49.7; // degrees obviously
    public static final double horizontalFOV = 63.3;
    public static final int horPixles = 1280;
  }

  public static final class Limelight3GConstants {
    public static final double verticalFOV = 49.7; // degrees obviously
    public static final double horizontalFOV = 63.3;
    public static final int horPixles = 1280;
  }

  public static final class Limelight4Constants {
    public static final double verticalFOV = 56.2; // degrees obviously
    public static final double horizontalFOV = 82;
    public static final int horPixles = 1280;
  }

  public static final class LimelightConstants {
    public static final String name = "limelight-front";
    public static final VisionObservation.LLTYPE limelightType = LLTYPE.LL3;
    public static final double limelightMountHeight = Units.inchesToMeters(20.5);
    public static final int detectorPiplineIndex = 1;
    public static final int apriltagPipelineIndex = 0;
    public static final double filterTimeConstant = 0.1; // in seconds, inputs occuring over a time period
    // significantly shorter than this will be thrown out
    public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    public static final limelightConstants constants = new limelightConstants(
        name,
        limelightType,
        Limelight4Constants.verticalFOV,
        Limelight4Constants.horizontalFOV,
        limelightMountHeight,
        detectorPiplineIndex,
        apriltagPipelineIndex,
        Limelight4Constants.horPixles,
        visionMeasurementStdDevs);

    public static final String ip = "10.1.77.17"; //TODO: CHANGE TO 2170 Limelight IDS
  }

  public static final class ElevatorConstants{
    public static final int elevatorMasterId = 27;
    public static final int elevatorFollowerId = 25;
    public static final int elevatorMasterCancoderId = 0;
    public static final int elevatorFollowerCancoderId = 0;
    public static final String canbus = "rio";
  }
}
