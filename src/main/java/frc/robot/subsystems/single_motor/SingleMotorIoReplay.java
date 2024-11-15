package frc.robot.subsystems.single_motor;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;

public class SingleMotorIoReplay implements SingleMotorIo {

    @Override
    public void updateInputs(SingleMotorIoInputs inputs) {
        // No functionality.
    }

    public Measure<Temperature> getMaxSafeTemperature() {
        return Units.Celsius.of(Double.MAX_VALUE);
    }

    public Measure<Temperature> getTemperature() {
        return Units.Celsius.of(0);
    }

    @Override
    public void setVelocity(double velocity) {
        // No functionality.
    }
}
