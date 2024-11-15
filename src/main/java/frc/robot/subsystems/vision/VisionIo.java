package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.Vision.VisionInputs;

public interface VisionIo {
    public void updateInputs(VisionInputs inputs);

    public String name();
}
