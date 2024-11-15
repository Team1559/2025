package frc.robot.subsystems.led;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

    // ========================= Class Level =========================
    private static int dynamicColorCounter = 0;

    /**
     * Takes dynamic pattern and scrolls colors by 1.
     * 
     * @param pattern         An Array of {@link Color}s to be shifted.
     * @param isScrollFowards Shifts colors fowads w.hen {@code true} backwards when
     *                        {@code false}
     * @param updateDelay     decreases the number of updates based on update delay,
     *                        updates every nth attempt.
     * @return Shifted array of colors.
     */
    public static Color[] scrollPattern(Color[] pattern, boolean isScrollFowards, int updateDelay) {
        Color[] tempArray = pattern;
        if (dynamicColorCounter % updateDelay == 0) {
            tempArray = new Color[pattern.length];
            if (isScrollFowards) {
                for (int i = 0; i < pattern.length; i++) {
                    if (i == pattern.length - 1) {
                        tempArray[0] = pattern[pattern.length - 1];
                    } else {
                        tempArray[i + 1] = pattern[i];
                    }
                }
            } else {
                for (int i = 0; i < pattern.length; i++) {
                    if (i == 0) {
                        tempArray[tempArray.length - 1] = pattern[0];
                    } else {
                        tempArray[i - 1] = pattern[i];
                    }
                }
            }
        }
        return tempArray;
    }

    // ========================= Object Level =========================
    private boolean isDynamicPatternFowards;
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;
    private Color[] dynamicPattern;

    /**
     * Initialize the {@link AddressableLED}, {@link AddresableLEDBuffer}, and
     * starts LEDs.
     */
    public Leds() {
        addressableLED = new AddressableLED(CONSTANTS.getLedPort());
        addressableLED.setLength(CONSTANTS.getLedLenth());
        ledBuffer = new AddressableLEDBuffer(CONSTANTS.getLedLenth());
        addressableLED.start();
    }

    @Override
    public void periodic() {
        if (dynamicPattern != null) {
            setStaticPatternHelper(dynamicPattern);
            dynamicColorCounter++;
            dynamicPattern = scrollPattern(dynamicPattern, isDynamicPatternFowards, 3);
        }
    }

    // ========================= Functions =========================
    /**
     * Increase or decreases the brightness of the colors currently set to the
     * {@link AddressableLEDBuffer} by 15%.
     * 
     * @param isDimming Decreases brightness when {@code true} and increases when
     *                  {@code false}.
     */
    public void changeBrightness(boolean isDimming) {
        double factor = isDimming ? .85 : 1.15;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            Color currentColor = ledBuffer.getLED(i);
            ledBuffer.setLED(i, new Color(currentColor.red * factor, currentColor.green * factor,
                    currentColor.blue * factor));
        }
        addressableLED.setData(ledBuffer);
    }

    /**
     * Disables dynamic pattern, does not change lights.
     */
    private void disableDynamicPattern() {
        dynamicPattern = null;
    }

    /**
     * Sets a pattern that scrolls either fowards or backwards.
     * <i>Notes:</i>
     * </p>
     * <ul>
     * <li>If the patters does not fit evenly into the LEDs, it will be
     * truncated.</li>
     * <li>{@link Color#kblack} can be used to sparate the poattern.</li>
     * </ul>
     * 
     * @param pattern                 Array of {@link Color}s to be set and scrolled
     *                                through.
     * @param isDynamicPatternFowards Scroll fowards when {@code true}, backwards
     *                                when {@code false}.
     */
    public void setDynamicPattern(Color[] pattern, boolean isDynamicPatternFowards) {
        dynamicPattern = pattern;
        this.isDynamicPatternFowards = isDynamicPatternFowards;
    }

    /**
     * Sets all lights to a static monocolor.
     * 
     * @param color {@link Color} the lights are being set to.
     */
    public void setColor(Color color) {
        disableDynamicPattern();
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        addressableLED.setData(ledBuffer);
    }

    public void setAllianceColor() {
        disableDynamicPattern();
        if (CONSTANTS.getAlliance() == Alliance.Blue) {
            setColor(Color.kBlue);
        } else {
            setColor(Color.kRed);
        }
    }

    /**
     * Sets all lights to a static multicolor pattern. This pattern will be repeated
     * arross the LEDs.
     * <p>
     * <i>Notes:</i>
     * </p>
     * <ul>
     * <li>If the patters does not fit evenly into the LEDs, it will be
     * truncated.</li>
     * <li>{@link Color#kblack} can be used to sparate the poattern.</li>
     * </ul>
     * 
     * @param pattern Array of {@link Color}s the lights are being set to.
     */
    public void setStaticPattern(Color[] pattern) {
        disableDynamicPattern();
        setStaticPatternHelper(pattern);
    }

    private void setStaticPatternHelper(Color[] pattern) {
        if (pattern.length == 0) {
            throw new RuntimeException("Pattern size may not be 0");
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, pattern[i % pattern.length]);
        }
        addressableLED.setData(ledBuffer);
    }

    /**
     * Disables dynamic patterns, turns off lights.
     */
    public void turnOff() {
        disableDynamicPattern();
        setColor(Color.kBlack);
    }

    // ========================= Commands =========================
    /**
     * Dims/Brightens the lights
     * 
     * @param isDimming are lights being dimmed or brightened
     * @return
     */
    public Command changeBrightnessCommand(boolean isDimming) {
        return new InstantCommand(() -> changeBrightness(isDimming), this);
    }

    /**
     * Set the lights to a scrolling pattern
     * 
     * @param pattern                 Pattern the LEDs are being set to
     * @param isDynamicPatternFowards is the pattern scrolling fowards or backwards
     * @return
     */
    public Command setDynamicPatternCommand(Color[] pattern, boolean isDynamicPatternFowards) {
        return new InstantCommand(() -> setDynamicPattern(pattern, isDynamicPatternFowards), this);
    }

    /**
     * Set color of the LEDs
     * 
     * @param color Color LEDs are being set to
     * @return
     */
    public Command setColorCommand(Color color) {
        return new RunCommand(() -> setColor(color), this);
    }

    /**
     * Sets a static patttern to the LEDs
     * 
     * @param subsystem LEDs being set
     * @param pattern   Pattern being set to the LEDs
     * @return
     */
    public Command setStaticPatternCommand(Color[] pattern) {
        return new InstantCommand(() -> setStaticPattern(pattern), this);
    }

    public Command setAllianceColorCommand() {
        return new InstantCommand(this::setAllianceColor, this);
    }

    public Command turnOffCommand() {
        return new InstantCommand(this::turnOff, this);
    }

    public Command disableDynamiPatternCommand() {
        return new InstantCommand(this::disableDynamicPattern, this);
    }
}
