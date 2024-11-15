package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIoPigeon2 implements GyroIo {

    private final Pigeon2 pigeon;

    private final StatusSignal<Double> yaw;
    private final StatusSignal<Double> yawVelocity;

    public GyroIoPigeon2(int deviceId, String canbus) {

        if (canbus == null) {
            pigeon = new Pigeon2(deviceId);
        } else {
            pigeon = new Pigeon2(deviceId, canbus);
        }

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100.0);
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIoInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble()).plus(Rotation2d.fromDegrees(0));
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }
}