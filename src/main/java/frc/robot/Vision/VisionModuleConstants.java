package frc.robot.Vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class VisionModuleConstants {
    public final String name;
    public final double verticalFOV;
    public final double horizontalFOV;
    public final double limelightMountHeight;
    public final VisionModuleType moduleType; 
    public final int pipelineIndex;
    public final double filterTimeConstant; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
    public final Vector<N3> visionMeasurementStdDevs;
    public final int movingAverageNumTaps;
    public final int horPixles;
    public final int vertPixels;



    /**
     * Limelight Constants to be used when creating vision modules.
     * @param name
     * @param verticalFOV
     * @param horizontalFov
     * @param limelightmountheight
     * @param moduleType
     * @param pipelineIndex
     * @param horpixels
     * @param horpixels
     * @param filterTimeConstant
     * @param visionMeasurementStdDevs
     * @param movingAverageNumTaps
     */

    public VisionModuleConstants(String name, double verticalFOV, double horizontalFOV, double limelightMountHeight, VisionModuleType moduleType, int pipelineIndex,
    int horPixles, int vertPixels, double filterTimeConstant, Vector<N3> visionMeasurementStdDevs, int movingAverageNumTaps) {
        this.name = name;
        this.verticalFOV = verticalFOV; //degrees obviously
        this.horizontalFOV = horizontalFOV;
        this.limelightMountHeight = limelightMountHeight;
        this.moduleType = moduleType; 
        this.pipelineIndex = pipelineIndex;
        this.horPixles = horPixles;
        this.vertPixels = vertPixels;
        this.filterTimeConstant = filterTimeConstant; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
        this.movingAverageNumTaps = movingAverageNumTaps;
    }
}
