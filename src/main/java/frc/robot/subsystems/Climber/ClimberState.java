package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberState {
    public Rotation2d rotations = Rotation2d.fromDegrees(0);
    public ClimberState(Rotation2d rotations) {
        this.rotations = rotations;
    }
}