package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.Vision.VisionInputs;

public class VisionIoSimAndReplay implements VisionIo {
    public void updateInputs(VisionInputs inputs) {
        // Don't need to do anything here.
    }

    public String name() {
        return "sim";
    }
}
