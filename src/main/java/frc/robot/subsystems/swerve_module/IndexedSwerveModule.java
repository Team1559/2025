package frc.robot.subsystems.swerve_module;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;

public class IndexedSwerveModule {

    private final SwerveModuleIo io;
    private final SwerveModuleIoInputsAutoLogged inputs = new SwerveModuleIoInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint; // Setpoint for closed loop control, null for open loop.
    private Double speedSetpoint; // Setpoint for closed loop control, null for open loop.
    private Rotation2d turnRelativeOffset; // Relative + Offset = Absolute.
    private double lastPositionMeters; // Used for delta calculation.

    public IndexedSwerveModule(SwerveModuleIo io, int index) {

        this.io = io;
        this.index = index;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (CONSTANTS.getCurrentOperatingMode()) {
            case REAL_WORLD:
            case LOG_REPLAY:
                driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                driveFeedback = new PIDController(0.05, 0.0, 0.0);
                turnFeedback = new PIDController(7.0, 0.0, 0.0);
                break;
            case SIMULATION:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                throw new RuntimeException(
                        "Unknown Run Mode: " + CONSTANTS.getCurrentOperatingMode());
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // On first cycle, reset relative turn encoder.
        // Wait until absolute angle is nonzero in case it wasn't initialized yet.
        if (turnRelativeOffset == null && inputs.cancoderOffsetPosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.cancoderOffsetPosition.minus(inputs.steerMotorPosition);
        }

        // Run closed loop turn control.
        if (angleSetpoint != null) {

            io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

            // Run closed loop drive control.
            // Only allowed if closed loop turn control is running.
            if (speedSetpoint != null) {

                // Scale velocity based on turn error.
                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

                // Run drive controller/
                double velocityRadPerSec = adjustSpeedSetpoint / CONSTANTS.getWheelRadius().in(Meters);
                io.setDriveVoltage(driveFeedforward.calculate(velocityRadPerSec)
                        + driveFeedback.calculate(inputs.driveMotorVelocityRadPerSec, velocityRadPerSec));
            }
        }
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized
     * state.
     */
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null

        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedState.angle;
        speedSetpoint = -optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     */
    public void runCharacterization(double volts) {
        // Closed loop turn control
        angleSetpoint = new Rotation2d();

        // Open loop drive control
        io.setDriveVoltage(volts);
        speedSetpoint = null;
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        // Disable closed loop control for turn and drive
        angleSetpoint = null;
        speedSetpoint = null;
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.steerMotorPosition.plus(turnRelativeOffset);
        }
    }

    /**
     * @return The Temperature of the hottest motor.
     */
    public Measure<Temperature> getMaxTemperature() {
        return inputs.driveMotorTemp.gt(inputs.steerMotorTemp) ? inputs.driveMotorTemp : inputs.steerMotorTemp;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.driveMotorPositionRad * CONSTANTS.getWheelRadius().in(Meters);
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveMotorVelocityRadPerSec * CONSTANTS.getWheelRadius().in(Meters);
    }

    /** Returns the module position. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module position delta since the last call to this method. */
    public SwerveModulePosition getPositionDelta() {
        SwerveModulePosition delta = new SwerveModulePosition(getPositionMeters() - lastPositionMeters, getAngle());
        lastPositionMeters = getPositionMeters();
        return delta;
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        return inputs.driveMotorVelocityRadPerSec;
    }

    public boolean isTemperatureTooHigh() {
        // 90% Buffer.
        return getMaxTemperature().gt(io.getMaxSafeMotorTemperature().times(CONSTANTS.SAFE_MOTOR_TEMPERATURE_BUFFER));
    }
}