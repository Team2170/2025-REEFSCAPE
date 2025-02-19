package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ElevatorState {
    CORAL_L1(Rotation2d.fromRotations(0)), // TODO change these to the appropriate heights of scoring
    CORAL_L2(Rotation2d.fromRotations(1)),
    CORAL_L3(Rotation2d.fromRotations(2)),
    CORAL_L4(Rotation2d.fromRotations(3)),
    HUMAN_PLAYER(Rotation2d.fromRotations(2)); // TODO change this to the appropriate height for feeding coral into
                                               // robot

    ElevatorState(Rotation2d pos) {
        this.pos = pos;
    }

    public Rotation2d pos;
}