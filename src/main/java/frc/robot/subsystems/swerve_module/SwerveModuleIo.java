package frc.robot.subsystems.swerve_module;

import static edu.wpi.first.units.Units.Celsius;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;

public interface SwerveModuleIo {

    @AutoLog
    static class SwerveModuleIoInputs {

        public Rotation2d cancoderOffsetPosition = new Rotation2d();
        public Rotation2d cancoderAbsolutePosition = new Rotation2d();

        public double driveMotorPositionRad;
        public double driveMotorVelocityRadPerSec;
        public double driveMotorAppliedVolts;
        public double driveMotorCurrentAmps;
        public int driveMotorFaults;
        public Measure<Temperature> driveMotorTemp = Celsius.zero();

        public Rotation2d steerMotorPosition = new Rotation2d();
        public double steerMotorVelocityRadPerSec;
        public double steerMotorAppliedVolts;
        public double steerMotorCurrentAmps;
        public int steerMotorFaults;
        public Measure<Temperature> steerMotorTemp = Celsius.zero();
    }

    /**
     * @return The maximum safe operating temperature of these motors.
     */
    public Measure<Temperature> getMaxSafeMotorTemperature();

    /** Run the drive motor at the specified voltage. */
    public void setDriveVoltage(double volts);

    /** Run the turn motor at the specified voltage. */
    public void setTurnVoltage(double volts);

    /** Updates the set of loggable inputs. */
    public void updateInputs(SwerveModuleIoInputs inputs);
}