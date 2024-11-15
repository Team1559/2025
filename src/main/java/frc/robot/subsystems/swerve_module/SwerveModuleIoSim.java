package frc.robot.subsystems.swerve_module;

import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO.
 *
 * <p>
 * Uses two flywheel sims for the drive and turn motors, with the absolute
 * position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide
 * a decent
 * approximation for the behavior of the module.
 */
public class SwerveModuleIoSim implements SwerveModuleIo {

    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts;
    private double turnAppliedVolts;

    @Override
    public Measure<Temperature> getMaxSafeMotorTemperature() {
        return Celsius.of(Double.MAX_VALUE);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    @Override
    public void updateInputs(SwerveModuleIoInputs inputs) {

        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.cancoderAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.cancoderOffsetPosition = inputs.cancoderAbsolutePosition.plus(turnAbsoluteInitPosition);

        inputs.driveMotorPositionRad = driveSim.getAngularPositionRad();
        inputs.driveMotorVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveMotorAppliedVolts = driveAppliedVolts;
        inputs.driveMotorCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.steerMotorPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.steerMotorVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.steerMotorAppliedVolts = turnAppliedVolts;
        inputs.steerMotorCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    }
}