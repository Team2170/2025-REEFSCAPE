package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
public class CTREConfigs {

    /* Configure CANcoder to zero the magnet appropriately */
    //function essentially sets everything to factory default
    public static void basicConfiguration(CANcoder encoder, TalonFX motor, int encoderId, int motorId) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.withMagnetOffset(0.4);
        encoder.getConfigurator().apply(encoderConfig);

        motorConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // climberMotorFXConfig.Feedback.SensorToMechanismRatio = 1.0;
        motor.getConfigurator().apply(motorConfig);
    }
}
