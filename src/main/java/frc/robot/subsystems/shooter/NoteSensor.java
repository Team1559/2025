package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class NoteSensor extends SubsystemBase {

    @AutoLog
    static class NoteSensorInputs {
        public boolean isObjectDetected;
        public boolean isSwitchLeftPressed;
        public boolean isSwitchRightPressed;
    }

    private NoteSensorInputsAutoLogged inputs = new NoteSensorInputsAutoLogged();
    private final DigitalInput limitSwitchLeft;
    private final DigitalInput limitSwitchRight;

    public NoteSensor(int leftChannel, int rightChannel) {
        limitSwitchLeft = new DigitalInput(leftChannel);
        limitSwitchRight = new DigitalInput(rightChannel);
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/NoteSensor", inputs);
    }

    private void updateInputs() {
        inputs.isSwitchLeftPressed = !limitSwitchLeft.get();
        inputs.isSwitchRightPressed = !limitSwitchRight.get();
        inputs.isObjectDetected = inputs.isSwitchLeftPressed || inputs.isSwitchRightPressed;
    }

    /**
     * Check if limit switch is activated
     * 
     * @return limit switch state;
     */
    public boolean isObjectDetected() {
        return inputs.isObjectDetected;
    }

    public boolean isObjectNotDetected() {
        return !isObjectDetected();
    }

    // ========================= Commands =========================
    public Command waitForObjectCommandSwitch() {
        return new WaitUntilCommand(this::isObjectDetected);
    }

    public Command waitForNoObjectCommandSwitch() {
        return new WaitUntilCommand(this::isObjectNotDetected);
    }
}
