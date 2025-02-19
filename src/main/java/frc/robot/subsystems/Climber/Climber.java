package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.module.ClimberModule;

public class Climber extends SubsystemBase {
    private ClimberModule module;

    public Climber() {
        module = new ClimberModule();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber " + ClimberConstants.climberId + " Encoder (rot)",
                module.getState().rotations.getRotations());
    }

    public ClimberModule getModule() {
        return module;
    }

    public void stopClimber() {
        module.stop();
    }

    /**
     * Extends the module given a desired state components of speed and motor
     * rotations.
     */
    public void setClimberPosition(double position) {
        module.setDesiredState(new ClimberState(Rotation2d.fromRotations(position)));
    }

    public Command testInitialization() {
        double pos = 0;
        Command extend = new InstantCommand(() -> setClimberPosition(pos)).withTimeout(2);
        Command retract = new InstantCommand(() -> setClimberPosition(-pos)).withTimeout(2);
        Command stop = new InstantCommand(() -> stopClimber()).withTimeout(2);
        Command composition = extend.andThen(stop).andThen(retract).andThen(stop);
        composition.setName("ClimberTest");
        return composition;
    }
    //TODO fix the commands appropriately to fit what their functionality should be for the game
    public Command stowClimb() {
        double timeout = 0.25;
        double pos = 1;
        Command pivot_cw = new InstantCommand(() -> setClimberPosition(-pos)).withTimeout(timeout);
        return pivot_cw;
    }

    public Command extendToClimb() {
        double timeout = 0.25;
        double pos = 1;
        Command pivot_ccw = new InstantCommand(() -> setClimberPosition(pos)).withTimeout(timeout);
        return pivot_ccw;
    }
}
