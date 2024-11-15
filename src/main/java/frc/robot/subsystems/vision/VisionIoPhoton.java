package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.VisionInputs;

public class VisionIoPhoton implements VisionIo {
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final PoseStrategy POSE_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;
    private final double AMBIGUITY_THRESHOLD = 0.2;
    private final double MAX_DISTANCE = 9.0;

    public VisionIoPhoton(String cameraName, String fieldName, Transform3d cameraToRobot) {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, POSE_STRATEGY,
                this.camera, cameraToRobot);

    }

    public String name() {
        return this.camera.getName();
    }

    public void updateInputs(VisionInputs inputs) {
        inputs.havePose = false;
        inputs.pose = new Pose2d();
        inputs.timestamp = 0;

        Optional<EstimatedRobotPose> optionalPoseEstimate = poseEstimator.update();
        if (!optionalPoseEstimate.isPresent()) {
            return;
        }
        EstimatedRobotPose poseEstimate = optionalPoseEstimate.get();

        PhotonPipelineResult cameraResult = camera.getLatestResult();
        if (!cameraResult.hasTargets()) {
            return;
        }

        PhotonTrackedTarget target = cameraResult.getBestTarget();
        if (target == null) {
            return;
        }

        if (target.getPoseAmbiguity() > AMBIGUITY_THRESHOLD) {
            return;
        }

        double distanceToTarget = target
                .getBestCameraToTarget()
                .getTranslation()
                .toTranslation2d()
                .getNorm();

        if (distanceToTarget > MAX_DISTANCE) {
            return;
        }
        // This strategy is from 2023: trust more distant targets less.
        // (maybe this should be a function of distance instead of the distance?)
        double[] stdevs = { distanceToTarget, distanceToTarget, distanceToTarget };

        inputs.havePose = true;
        inputs.pose = poseEstimate.estimatedPose.toPose2d();
        inputs.estimateStdDevs = stdevs;
        inputs.timestamp = poseEstimate.timestampSeconds;
    }
}
