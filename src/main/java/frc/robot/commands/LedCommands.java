package frc.robot.commands;

import java.time.Duration;
import java.time.LocalTime;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.led.Leds;

public class LedCommands {

    /** Private to prevent instantiation. */
    private LedCommands() {
    }

    /**
     * Blink the LEDs to specified Color and then return to Alliance color
     * 
     * @param leds  leds being set
     * @param color color being blinked to
     * @return Blink Command
     */
    public static Command blinkCommand(Leds leds, Color color) {
        Duration WAIT_TIME = Duration.ofMillis(500);
        Command blinkCommand = new Command() {
            LocalTime startTime;

            @Override
            public void initialize() {
                leds.setColor(color);
                startTime = LocalTime.now();
            }

            @Override
            public boolean isFinished() {
                Duration timeWaited = Duration.between(startTime, LocalTime.now());
                return timeWaited.compareTo(WAIT_TIME) >= 0;
            }
        };
        blinkCommand.addRequirements(leds);
        return blinkCommand;
    }

    public static Command defaultLedCommand(Leds leds) {
        return Commands.run(() -> {
            leds.setAllianceColor();
        }, leds);
    }
}
