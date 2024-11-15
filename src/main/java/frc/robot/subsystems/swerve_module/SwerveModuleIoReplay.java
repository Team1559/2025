package frc.robot.subsystems.swerve_module;

import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;

/**
 * No funclionality provided because none needed for replay.
 */
public class SwerveModuleIoReplay implements SwerveModuleIo {

    @Override
    public Measure<Temperature> getMaxSafeMotorTemperature() {
        return Celsius.of(Double.MAX_VALUE);
    }

    @Override
    public void setDriveVoltage(double volts) {
        // No functionality.
    }

    @Override
    public void setTurnVoltage(double volts) {
        // No functionality.
    }

    @Override
    public void updateInputs(SwerveModuleIoInputs inputs) {
        // No functionality.
    }
}