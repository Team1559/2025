package frc.robot.subsystems.base;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gyro.GyroIo;
import frc.robot.subsystems.gyro.GyroIoInputsAutoLogged;
import frc.robot.subsystems.swerve_module.IndexedSwerveModule;
import frc.robot.subsystems.swerve_module.SwerveModuleIo;
import frc.robot.util.LocalAdStarAk;

public class DriveBase extends SubsystemBase {

    // ========================= Class Level ===================================

    public enum WheelModuleIndex {
        /** 0 */
        FRONT_LEFT(0),
        /** 1 */
        FRONT_RIGHT(1),
        /** 2 */
        BACK_LEFT(2),
        /** 3 */
        BACK_RIGHT(3);

        public final int value;

        private WheelModuleIndex(int value) {
            this.value = value;
        }
    }

    private static final double ENCODER_STDDEV = 0.01;

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(CONSTANTS.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        CONSTANTS.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(CONSTANTS.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        -CONSTANTS.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(-CONSTANTS.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        CONSTANTS.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(-CONSTANTS.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        -CONSTANTS.getWheelDistanceLeftToRight().in(Meters) / 2.0)
        };
    }

    // ========================= Object Level ==================================

    private final GyroIo gyroIO;
    private final GyroIoInputsAutoLogged gyroInputs = new GyroIoInputsAutoLogged();
    private final IndexedSwerveModule[] modules = new IndexedSwerveModule[4];

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    public final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModulePosition[] modulePositions;
    private Translation2d lastPosition;

    public DriveBase(GyroIo gyroIo,
            SwerveModuleIo flModuleI,
            SwerveModuleIo frModuleIo,
            SwerveModuleIo blModuleIo,
            SwerveModuleIo brModuleIo) {

        this.gyroIO = gyroIo;
        modules[WheelModuleIndex.FRONT_LEFT.value] = new IndexedSwerveModule(flModuleI,
                WheelModuleIndex.FRONT_LEFT.value);
        modules[WheelModuleIndex.FRONT_RIGHT.value] = new IndexedSwerveModule(frModuleIo,
                WheelModuleIndex.FRONT_RIGHT.value);
        modules[WheelModuleIndex.BACK_LEFT.value] = new IndexedSwerveModule(blModuleIo,
                WheelModuleIndex.BACK_LEFT.value);
        modules[WheelModuleIndex.BACK_RIGHT.value] = new IndexedSwerveModule(brModuleIo,
                WheelModuleIndex.BACK_RIGHT.value);

        modulePositions = new SwerveModulePosition[4];
        updateModulePositions();

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroInputs.yawPosition, modulePositions,
                new Pose2d(0, 0, gyroInputs.yawPosition),
                VecBuilder.fill(ENCODER_STDDEV, ENCODER_STDDEV, ENCODER_STDDEV),
                VecBuilder.fill(1, 1, 1)); // placeholder, will be filled in by vision

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(CONSTANTS.getMaxLinearSpeed().in(MetersPerSecond),
                        CONSTANTS.getDriveBaseWheelRadius().in(Meters), new ReplanningConfig()),
                // Flips path if aliance is on red side.
                () -> CONSTANTS.getAlliance() != CONSTANTS.getDefaultAllianceForAuto()
                        && CONSTANTS.shouldFlipPathIfAssignedAllianceIsNotDefault(),
                this);
        Pathfinding.setPathfinder(new LocalAdStarAk());
        PathPlannerLogging.setLogActivePathCallback(
                activePath -> Logger.recordOutput("Odometry/Trajectory",
                        activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
                targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        // TODO: Figure out why the robot is not starting at 0,0.
        setPose(new Pose2d(new Translation2d(),
                CONSTANTS.getAlliance() == Alliance.Blue ? Rotation2d.fromDegrees(180) : new Rotation2d()));
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        getPose(); // Logs Robot Estimated Position; (TODO: Is this needed?)
        getSpeed(); // same, revisit later
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (IndexedSwerveModule module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (IndexedSwerveModule module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints []");
            Logger.recordOutput("SwerveStates/SetpointsOptimized []");
        }

        // Update odometry
        updateModulePositions();
        poseEstimator.update(gyroInputs.yawPosition, modulePositions);
        getPose(); // Logs Robot Estimated Positio;
    }

    // ========================= Functions =====================================

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "EstimatedPosition")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "EstimatedSpeed")
    public double getSpeed() {
        if (lastPosition == null) {
            lastPosition = getPose().getTranslation();
        }
        Translation2d current = getPose().getTranslation();
        Translation2d delta = current.minus(lastPosition);
        lastPosition = current;
        return delta.getNorm() / 0.02;
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public boolean isTemperatureTooHigh() {
        for (IndexedSwerveModule module : modules) {
            if (module.isTemperatureTooHigh()) {
                return true;
            }
        }
        return false;
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(volts);
        }
    }

    public void resetFieldOrientation() {
        poseEstimator.addVisionMeasurement(
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()),
                Timer.getFPGATimestamp(), VecBuilder.fill(0, 0, 0));
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {

        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CONSTANTS.getMaxLinearSpeed());

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(gyroInputs.yawPosition, modulePositions, pose);
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    // ========================= Calculations ==================================

    /**
     * This gets the rotation from the current position to the target position, and
     * it's trying to point the front of the robot to the target.
     *
     * @param target
     * @return
     */
    public Rotation2d getRotationToTarget(Translation2d target) {
        Pose2d currentPose = getPose();
        Translation2d deltaTranslation = target.minus(currentPose.getTranslation());
        Rotation2d deltaAngle = deltaTranslation.getAngle();
        return deltaAngle.minus(currentPose.getRotation());
    }

    // ========================= Commands ======================================

    // TODO: Add other Functions as Commands.

    /** Uses a RunCommand. */
    public Command runVelocityCommand(ChassisSpeeds speeds) {
        return new RunCommand(() -> this.runVelocity(speeds), this);
    }

    // ========================= Helper Methods ================================
    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private void updateModulePositions() {
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
    }

    // ========================= Commands =====================================

    public Command resetFieldOrientationCommand() {
        return new InstantCommand(this::resetFieldOrientation);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop);
    }

    // TODO: Add remaining Instant Commands.
}