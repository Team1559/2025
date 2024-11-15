package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIo {

    @AutoLog
    public static class GyroIoInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
    }

    public void updateInputs(GyroIoInputs inputs);
}