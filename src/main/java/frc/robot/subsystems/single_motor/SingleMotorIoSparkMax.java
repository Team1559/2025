package frc.robot.subsystems.single_motor;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;

public abstract class SingleMotorIoSparkMax implements SingleMotorIo {

    protected final CANSparkMax motor;
    protected double velocity;
    protected boolean inverted;

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId  Motor CAN ID
     * @param inverted True if the motor direction should be inverted
     */
    public SingleMotorIoSparkMax(int motorId, boolean inverted, double kp, double ki, double kd, double ff) {
        motor = new CANSparkMax(motorId, MotorType.kBrushless);
        motor.setInverted(false); // TODO - randomly flips back
        this.inverted = inverted;
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
        motor.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
        motor.getPIDController().setP(kp);
        motor.getPIDController().setI(ki);
        motor.getPIDController().setD(kd);
        motor.getPIDController().setFF(ff);
    }

    public void updateInputs(SingleMotorIoInputs inputs) {
        inputs.appliedOutput = motor.getAppliedOutput();
        inputs.outputCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.faults = motor.getFaults();
        inputs.velocity = motor.getEncoder().getVelocity();
    }

    public Measure<Temperature> getTemperature() {
        return Units.Celsius.of(motor.getMotorTemperature());
    }

    public void setVelocity(double velocity) {
        motor.getPIDController().setReference(inverted ? -velocity : velocity, CANSparkMax.ControlType.kVelocity);
    }
}
