package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightBetter extends SubsystemBase {
    private String limelightName;
    private Supplier<Rotation3d> rotSupp;
    private Rotation3d currentRot = null;
    private Timer l_timer;
    private double prevTime = 0;
    
    /** Enum for Controlling the Limelight's LEDs:
     * @param PipelineControl Sets LED mode to be controlled by the current pipeline
     * @param ForceOff Forces the LED to be off
     * @param ForceBlink Forces the LED to blink 
     * @param ForceOn Forces the LED to stay on
     */
    public enum LEDMode {
        PipelineControl,
        ForceOff,
        ForceBlink,
        ForceOn
    }

    /** Enum for Controlling the DriverStation's FRC NetworkTable Dashboard Camera View:
     * @param Standard Standard side-by-side stream mode
     * @param PiPMain Picture-in-Picture mode with secondary stream in the corner.
     * @param PiPSecondary Picture-in-Picture mode with primary stream in the corner
     */
    public enum StreamMode {
        Standard,
        PiPMain,
        PiPSecondary
    }

    /** Different types of double values accessible by limelight for the Target
     * @param TV Whether the limelight has any valid targets (0 or 1)
     * @param TX Horizontal Offset From Crosshair To Target in Degrees (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     * @param TY Vertical Offset From Crosshair To Target in Degrees(LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     * @param TXNC The horizontal offset from the principal pixel/point to the target in degrees.  This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param TYNC The vertical offset from the principal pixel/point to the target in degrees. This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param TA The target area as a percentage of the image (0-100%)
     * @param thor Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     * @param tvert Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */ 
    public enum TargetData {
        TV,
        TX,
        TY,
        TXNC,
        TYNC,
        TA,
        thor,
        tvert
    }

    /** Different types of double values accessible by limelight for the Target
     * @param 
     */ 
    public enum IDType {
        NeuralClassID,

    }
    
    /** All-Parameter constructor for limelight
     * @param limelightName Name of the limelight to be initialized
     * @param testMode If the robot is being used in competition or testing - enables viewing over USB if true
     * @param yawSupplier A lambda that supplies the yaw of the robot to the limelight.
     * @param LLPose A Pose2D representing the Limelight's offset from the center of the robot, where the origin is the robot's center.
     */
    public LimelightBetter(String limelightName, boolean testMode, Supplier<Rotation3d> rotSupplier, Pose3d LLPose) {
        this.limelightName = limelightName;
        if (testMode) {
            enableViaUSB();
        }
        this.rotSupp = rotSupplier;
        if (rotSupp != null)
            currentRot = rotSupp.get();
        //Offsets the limelight in the robot's space
        setNTValue("camerapose_robotspace_set", NetworkTableValue.makeDoubleArray(
        new double[] {
            LLPose.getX(), //X Position
            LLPose.getY(), //Y Position
            LLPose.getZ(), //Z Position
            LLPose.getRotation().getX(), //Roll
            LLPose.getRotation().getY(), //Pitch
            LLPose.getRotation().getZ() //Yaw
        }));
        l_timer.start();
    }

    /** Periodically updates gyro information on the limelight.
     * 
     */
    @Override
    public void periodic() {
        super.periodic();
        if (currentRot != null) {

            Rotation3d update = rotSupp.get();
            Rotation3d change = currentRot.minus(update);
            

            double period =  l_timer.get() - prevTime;
            prevTime = l_timer.get();

            //Avoid instrument errors
            if (Math.abs(change.getAngle()) > 720) {
                change.div(period);
                setNTValue("robot_orientation_set", NetworkTableValue.makeDoubleArray(
                    new double[] {
                        update.getZ(),
                        change.getZ(),
                        update.getY(),
                        change.getY(),
                        update.getX(),
                        change.getX()
                    }
                ));
                currentRot = update;
            }
        }
    }

    public String getLimelightName() {
        return limelightName;
    }

    public void setLimelightName(String limelightName) {
        this.limelightName = limelightName;
    }

    public static NetworkTableEntry getNetworkTableEntry(String table, String entryName) {
        return NetworkTableInstance.getDefault().getTable(table).getEntry(entryName);
    }

    public static void setNetworkTableEntry(String limelightName, String entryName, NetworkTableValue value) {
        getNetworkTableEntry(limelightName, entryName).setValue(value);
    }

    /**Enables Port-Forwarding so the Limelight can be used over USB*/
    public void enableViaUSB() {
        // Make sure you only configure port forwarding once in your robot code.
        // Do not place these function calls in any periodic functions
        int port = 5800;
        if (!limelightName.equals(limelightName))
            port += 10;
        for (; port <= 5809; port++) {
            PortForwarder.add(port, limelightName + ".local", port);
        }
    }


    /** Gets a value from the NetworkTable from the table corresponding to this limelight.
     * @param entryName Name of the entry to be taken from the NetworkTable
     * @return A 'NetworkTableEntry' containing the current value of the entry and functions to change that value.
     */
    public NetworkTableValue getNTValue(String entryName) {
        return NetworkTableInstance.getDefault().getTable(limelightName).getEntry(entryName).getValue();
    }

    /** Sets a value from the NetworkTable from the table corresponding to this limelight.
     * @param entryName Name of the entry to be taken from the NetworkTable
     * @return A 'NetworkTableEntry' containing the current value of the entry and functions to change that value.
     */
    public void setNTValue(String entryName, NetworkTableValue value) {
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry(entryName).setValue(value);
    }

    /**
     * Does the Limelight have a valid target?
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return True if a valid target is present, false otherwise
     */
    public void setLEDMode(LEDMode mode) {
        //TODO: Swap for makeDouble if needed
        setNTValue("ledMode", NetworkTableValue.makeInteger(mode.ordinal()));
    }

     /**
     * Controls the type of Stream Mode used by the limelight - Standard (camera), PiP w/ main cam big, and PiP w/ main cam shrunk.
     * @param limelightName Name of the Limelight camera
     */
    public void setStreamMode(StreamMode mode) {
        //TODO: Swap for makeDouble if needed
        setNTValue("stream", NetworkTableValue.makeInteger(mode.ordinal()));
    }

    /**
     * Types:
     * @param value The name of the intended data to be retrieved from the limelight (see Javadoc for TargetData)
     * @return The value as a double
     */
    public double get(TargetData value) {
        return get(value.name()).getDouble();
    }

    /**
     * Types:
     * @param value The name of the intended data to be retrieved from the limelight (see Javadoc for dType)
     * @return The value as a double
     */
    public NetworkTableValue get(String value) {
        //If network table value is 1, then the object is visible
        return getNTValue(value);
    }

    /**Does the Limelight have a valid target?
     * @return True if a valid target is present, false otherwise
     */
    public boolean getTV() {
        //If network table value is 1, then the object is visible
        return Math.abs(getNTValue("tv").getDouble() - 1.0) < 0.01;
    }
    /**
     * Sets limelight’s operation mode
     * @param vision True: Use as Vision processor
     * @param vision False: Use solely as Driver Camera (Increases exposure, disables vision processing)
     */
    public void setCameraMode(boolean vision) {
        setNTValue("camMode", NetworkTableValue.makeInteger(vision ? 0 : 1));
    }

    public void setPipeline(int pipeline) {
        setNTValue("pipeline", NetworkTableValue.makeInteger(pipeline));
    }

    public Pose2d getRobotPose2d() {
        Pose3d robbyPose = getRobotPose3d();
        return robbyPose != null ? robbyPose.toPose2d() : null;
    }


    /** Gets the robot's pose in 3d space as found in the Network Table for this limelight
     * @return a Pose3D representing the robot's pose in 3D space, or null if no data found
     */
    private Pose3d getRobotPose3d() {
        double[] values;
        if (rotSupp != null) 
            values = getNTValue("botpose_wpiblue").getDoubleArray();
        else 
            values = getNTValue("botpose_orb_wpiblue").getDoubleArray();
        return toPose3D(values);
    }

    /**
     * Gets the target's 3D pose with respect to the robot's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the target's position and orientation relative to the robot
     */
    public Pose3d getTargetPose3d_robotSpace() {
        double[] poseArray = getNTValue("targetpose_robotspace").getDoubleArray();
        return toPose3D(poseArray);
    }

    /**
     * Gets the target's 3D pose with respect to the camera's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the target's position and orientation relative to the camera
     */
    public Pose3d getTargetPose3d_CameraSpace() {
        double[] poseArray = getNTValue("targetpose_cameraspace").getDoubleArray();
        return toPose3D(poseArray);
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose3d object.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose3d object representing the pose
     */
    public static Pose3d toPose3D(double[] inData){
        if (inData == null || inData.length < 6) {
            return null;
        }
        return new Pose3d(
            new Translation3d(
                inData[0], 
                inData[1], 
                inData[2]
            ),
            new Rotation3d(
                Units.degreesToRadians(inData[3]), 
                Units.degreesToRadians(inData[4]),
                Units.degreesToRadians(inData[5])
            )
        );
    }

    /**
     * Converts a Pose3d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees.
     * 
     * @param pose The Pose3d object to convert
     * @return A 6-element array containing [x, y, z, roll, pitch, yaw]
     */
    public static double[] pose3dToArray(Pose3d pose) {
        double[] result = new double[6];
        result[0] = pose.getTranslation().getX();
        result[1] = pose.getTranslation().getY();
        result[2] = pose.getTranslation().getZ();
        result[3] = Units.radiansToDegrees(pose.getRotation().getX());
        result[4] = Units.radiansToDegrees(pose.getRotation().getY());
        result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
        return result;
    }
}
