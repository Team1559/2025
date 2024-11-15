package frc.robot.subsystems.climber;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    @AutoLog
    static class ClimberInputs { // TODO: Log everything that SingleMotorIo is.
        public double currentLeftPositionRotations;
        public double currentRightPositionRotations;
        public double currentAveragePositionRotations;
    }

    private final CANSparkMax motorL = new CANSparkMax(CONSTANTS.getClimberMotorIdLeft(), MotorType.kBrushless);
    private final CANSparkMax motorR = new CANSparkMax(CONSTANTS.getClimberMotorIdRight(), MotorType.kBrushless);

    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    public Climber() {
        motorL.setInverted(true);
        motorR.setInverted(false);
        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
        motorR.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
        motorL.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
        motorR.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
    }

    @Override
    public void periodic() {
        // Log Inputs.
        updateInputs();
        Logger.processInputs("Climber/Climber", inputs);
    }

    private void updateInputs() {

        inputs.currentLeftPositionRotations = motorL.getEncoder().getPosition();
        inputs.currentRightPositionRotations = motorR.getEncoder().getPosition();
        inputs.currentAveragePositionRotations = (inputs.currentLeftPositionRotations
                + inputs.currentRightPositionRotations) / 2;
    }

    public void setVoltage(double volts) {
        setVoltageLeft(volts);
        setVoltageRight(volts);
    }

    public void setVoltageLeft(double volts) {
        motorL.setVoltage(volts);
    }

    public void setVoltageRight(double volts) {
        motorR.setVoltage(volts);
    }

    public Command setVoltageUpCommand() {
        return new StartEndCommand(() -> setVoltage(11), () -> setVoltage(0), this);
    }

    public Command setLeftVoltageUpCommand() {
        return new StartEndCommand(() -> setVoltageLeft(11), () -> setVoltageLeft(0), this);
    }

    public Command setRightVoltageUpCommand() {
        return new StartEndCommand(() -> setVoltageRight(11), () -> setVoltageRight(0), this);
    }

    public Command setVoltageDownCommand() {
        return new StartEndCommand(() -> setVoltage(-11), () -> setVoltage(0), this);
    }

    public Command setLeftVoltageDownCommand() {
        return new StartEndCommand(() -> setVoltageLeft(-11), () -> setVoltageLeft(0), this);
    }

    public Command setRightVoltageDownCommand() {
        return new StartEndCommand(() -> setVoltageRight(-11), () -> setVoltageRight(0), this);
    }
}