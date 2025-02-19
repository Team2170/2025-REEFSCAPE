// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static final String canivorename = ""; // TODO add canivore name

  /**
   * Example of an inner class. One can "import static
   * [...].Constants.OIConstants.*" to gain access
   * to the constants contained within without having to preface the names with
   * the class, greatly
   * reducing the amount of text required.
   */
  public static final class OIConstants {
    // Example: the port of the driver's controller
    public static final int kDriverControllerPort = 0;
  }

  public static final class ClimberConstants {
    boolean isInverted;
    public static final int climberId = 0; // TODO change to proper id
    public static final int encoderId = 1; // TODO
  }

  public static final class ElevatorConstants {
    public static final int leftMotorId = 3;
    public static final int rightMotorId = 4;
    public static final int leftEncoderId = 5;
    public static final int rightEncoderId = 6;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0; //output to overcome gravity
    public static final double kS = 0.0;
  }

  public static final class GripperConstants {
    public static final int leftMotorId = 7;
    public static final int rightMotorId = 8;
    public static final int leftEncoderId = 9;
    public static final int rightEncoderId = 10;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0; //output to overcome friction
    public static final double kV = 0.0; //output per unit of requested velocity
  }
}
