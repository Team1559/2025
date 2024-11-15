package frc.robot.subsystems.single_motor;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSubsystem extends SubsystemBase {

    private final double DEFAULT_FORWARDS_VELOCITY;
    private final double DEFAULT_REVERSE_VELOCITY;

    private final SingleMotorIo io;
    private final SingleMotorIoInputsAutoLogged inputs = new SingleMotorIoInputsAutoLogged();

    private double appliedVelocity;

    /**
     * Create a new subsystem for a single motor in velocity mode
     * 
     * @param name Name of subsystem
     * @param io   SingleMotorIO instance for the specific motor type
     */
    protected SingleMotorSubsystem(String name, SingleMotorIo io, double velocity) {
        this(name, io, velocity, velocity);

    }

    protected SingleMotorSubsystem(String name, SingleMotorIo io, double forwardsVelocity, double reverseVelocity) {

        super(name);

        this.io = io;
        this.DEFAULT_FORWARDS_VELOCITY = forwardsVelocity;
        this.DEFAULT_REVERSE_VELOCITY = reverseVelocity;
        this.appliedVelocity = 0;
    }

    @Override
    public void periodic() {

        // Set Velocitys.
        io.setVelocity(appliedVelocity);

        // Log Inputs.
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    // ========================= Functions =========================
    public boolean isTemperatureTooHigh() {
        // 90% Buffer.
        return io.getTemperature().gt(io.getMaxSafeTemperature().times(CONSTANTS.SAFE_MOTOR_TEMPERATURE_BUFFER));
    }

    public void reverse() {
        setVelocity(DEFAULT_REVERSE_VELOCITY);
    }

    public void setVelocity(double velocity) {
        appliedVelocity = velocity;
    }

    public void start() {
        setVelocity(DEFAULT_FORWARDS_VELOCITY);
    }

    public void stop() {
        setVelocity(0.0);
    }

    // ========================= Commands =========================

    public Command startCommand() {
        return new InstantCommand(this::start, this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public Command reverseCommand() {
        return new InstantCommand(this::reverse, this);
    }
}
