package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.Vision.VisionInputs;

public class VisionIoLimelight implements VisionIo {
    private static final double LINEAR_STD_DEV_RATIO = 0.5;
    private static final double ROTATION_STD_DEV = 1;

    private static final double FIELD_X_SIZE = 16.54;
    private static final double FIELD_Y_SIZE = 8.21;
    private static final double MAX_SPEED_FACTOR = 1.50;

    private final String cameraName;
    private Pose2d lastPose;
    private double lastPoseTime;

    private final DoubleSupplier speedSupplier;

    public VisionIoLimelight(String cameraName, DoubleSupplier speedSupplier) {
        this.cameraName = cameraName;
        this.speedSupplier = speedSupplier;

    }

    public String name() {
        return this.cameraName;
    }

    private boolean isPlausiblePose(Pose2d pose, double timestamp) {
        if (pose.getX() < 0 || pose.getX() > FIELD_X_SIZE) {
            return false;
        }
        if (pose.getY() < 0 || pose.getY() > FIELD_Y_SIZE) {
            return false;
        }
        if (lastPose != null) {
            Measure<Distance> distance = Meters.of(pose.getTranslation().getDistance(lastPose.getTranslation()));
            Measure<Time> time = Seconds.of(timestamp - lastPoseTime);
            Measure<Velocity<Distance>> speed = distance.per(time);
            if (speed.gt(CONSTANTS.getMaxLinearSpeed().times(MAX_SPEED_FACTOR))) {
                // Not possible for the robot to have moved that far that fast
                return false;
            }
        }
        return true;
    }

    public void updateInputs(VisionInputs inputs) {
        double[] data = LimelightHelpers.getBotPose_wpiBlue(cameraName);
        LimelightResults results = LimelightHelpers.getLatestResults(cameraName);
        if (data.length < 6 || (data[0] == 0 && data[1] == 0)) {
            inputs.havePose = false;
            inputs.pose = new Pose2d();
            inputs.timestamp = 0;
        } else {
            // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
            Translation2d t = new Translation2d(data[0], data[1]);
            Rotation2d r = Rotation2d.fromDegrees(data[5]);
            Pose2d pose = new Pose2d(t, r);
            double timestamp = Timer.getFPGATimestamp() - data[6] / 1000.0;

            if (!isPlausiblePose(pose, timestamp)) {
                inputs.havePose = false;
                inputs.pose = new Pose2d();
                inputs.timestamp = 0;
                return;
            }
            inputs.havePose = true;
            inputs.pose = pose;
            inputs.timestamp = timestamp;

            lastPose = pose;
            lastPoseTime = timestamp;

            double[] targetdata = LimelightHelpers.getTargetPose_CameraSpace(cameraName);
            inputs.distanceToTarget = Math.hypot(targetdata[0], targetdata[1]);
            double translationStdDev = (inputs.distanceToTarget * LINEAR_STD_DEV_RATIO
                    / Math.max(1, results.targetingResults.targets_Fiducials.length))
                    / Math.max(.5, speedSupplier.getAsDouble()) / 2;
            inputs.estimateStdDevs[0] = translationStdDev;
            inputs.estimateStdDevs[1] = translationStdDev;
            inputs.estimateStdDevs[2] = ROTATION_STD_DEV;

            // int[] tempFIDs = new int[results.targetingResults.targets_Fiducials.length];
            // for (int i = 0; i < results.targetingResults.targets_Fiducials.length; i++) {
            // tempFIDs[i] = (int) results.targetingResults.targets_Fiducials[i].fiducialID;
            // }
            // inputs.fIds = tempFIDs;
        }
    }
}
