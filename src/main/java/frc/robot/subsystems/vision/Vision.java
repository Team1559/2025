package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final VisionIo[] ios;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

    @AutoLog
    public static class VisionInputs {
        public VisionInputs() {
            estimateStdDevs = new double[3];
        }

        public boolean havePose = false;
        public Pose2d pose = new Pose2d();
        public double distanceToTarget;
        public double timestamp;
        public double[] estimateStdDevs = new double[3];
        // public int[] fIds = new int[0];
    }

    public Vision(SwerveDrivePoseEstimator poseEstimator, VisionIo... ios) {
        this.poseEstimator = poseEstimator;
        this.ios = ios;
    }

    @Override
    public void periodic() {
        for (VisionIo io : ios) {
            io.updateInputs(inputs);
            Logger.processInputs("Vision/" + io.name(), inputs);
            if (inputs.havePose) {
                double[] sd = inputs.estimateStdDevs;
                Vector<N3> sdVector = VecBuilder.fill(sd[0], sd[1], sd[2]);
                poseEstimator.addVisionMeasurement(inputs.pose, inputs.timestamp, sdVector);
            }
        }
    }
}
