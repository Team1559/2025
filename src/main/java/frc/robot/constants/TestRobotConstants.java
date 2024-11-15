package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;

import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;

public class TestRobotConstants extends AbstractConstants {

    // ==================== Methods (Ctrl + K, Ctrl + 8 to fold regions) =======
    // #region: --------------- Alliance ---------------------------------------
    public boolean shouldFlipPathIfAssignedAllianceIsNotDefault() {
        return true;
    }

    // #endregion

    // #region: --------------- Capability Flags -------------------------------
    @Override
    public boolean hasAimerSubsystem() {
        return false;
    }

    @Override
    public boolean hasClimberSubsystem() {
        return false;
    }

    @Override
    public boolean hasNoteSensorSubsystem() {
        return false;
    }

    @Override
    public boolean hasFeederSubsystem() {
        return false;
    }

    @Override
    public boolean hasFlywheelSubsystem() {
        return false;
    }

    @Override
    public boolean hasIntakeSubsystem() {
        return false;
    }

    @Override
    public boolean hasTraverserSubsystem() {
        return false;
    }

    @Override
    public boolean hasVisionSubsystem() {
        return true;
    }

    // #endregion

    // #region: --------------- Driving Configurations -------------------------
    public Measure<Velocity<Angle>> getMaxAngularSpeed() {
        return Radians.of(360).per(Second);
    }

    public Measure<Velocity<Distance>> getMaxLinearSpeed() {
        // TODO: Tune.
        return MetersPerSecond.of(10);
    }

    // #endregion

    // #region: --------------- Hardware ---------------------------------------
    // #region: ----- Aimer -----
    @Override
    public Tuple2<Rotation2d> getAimerAngleRange() {
        throw new UnsupportedOperationException("Unimplemented method 'getAimerAngleRange'");
    }

    @Override
    public Rotation2d getAimerEncoderOffset() {
        throw new UnsupportedOperationException("Unimplemented method 'getAimerEncoderOffset'");
    }

    @Override
    public PID getAimerPid() {
        throw new UnsupportedOperationException("Unimplemented method 'getAimerPid'");
    }

    // #endregion

    // #region: ----- Climber -----
    @Override
    public Measure<Distance> getClimberMaxHeight() {
        throw new UnsupportedOperationException("Unimplemented method 'getClimberMaxHeight'");
    }

    @Override
    public PID getClimberPid() {
        throw new UnsupportedOperationException("Unimplemented method 'getClimberPid'");
    }

    // #endregion
    // #region: ----- Feeder -----

    @Override
    public boolean isFeederMortorInverted() {
        throw new UnsupportedOperationException("Unimplemented method 'isFeederMortorInverted'");
    }

    @Override
    public double getFeederForwardVelocity() {
        throw new UnsupportedOperationException("Unimplemented method 'getFeederForwardVoltage'");
    }

    @Override
    public double getFeederReverseVelocity() {
        throw new UnsupportedOperationException("Unimplemented method 'getFeederReverseVoltage'");
    }

    // #endregion

    // #region: ----- Flywheel -----

    @Override
    public double getFlywheelForwardVoltage() {
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheelForwardVoltage'");
    }

    @Override
    public double getFlywheelReverseVoltage() {
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheelReverseVoltage'");
    }

    @Override
    public double flywheelSpinOffset() {
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheelMotorPowerDifferentialPercentage'");
    }

    // #endregion

    // #region: ----- Intake -----

    @Override
    public boolean isIntakeMortorInverted() {
        throw new UnsupportedOperationException("Unimplemented method 'isIntakeMortorInverted'");
    }

    @Override
    public double getIntakeForwardVelocity() {
        throw new UnsupportedOperationException("Unimplemented method 'getIntakeForwardVoltage'");
    }

    @Override
    public double getIntakeReverseVelocity() {
        throw new UnsupportedOperationException("Unimplemented method 'getIntakeReverseVoltage'");
    }

    // #endregion

    // #region: ----- LEDs -----
    @Override
    public int getLedLenth() {
        return 144;
    }

    // #endregion

    // #region: ----- roboRIO -----
    @Override
    public String getRoboRioSerialNumber() {
        return "03282BB6";
    }

    // #endregion

    // #region: ----- Swerve -----
    @Override
    public Map<WheelModuleIndex, Rotation2d> getSwerveModuleEncoderOffsets() {
        return new HashMap<>() {
            {
                put(WheelModuleIndex.FRONT_LEFT, Rotation2d.fromRotations(-0.300293));
                put(WheelModuleIndex.FRONT_RIGHT, Rotation2d.fromRotations(-0.228760));
                put(WheelModuleIndex.BACK_LEFT, Rotation2d.fromRotations(-0.238525));
                put(WheelModuleIndex.BACK_RIGHT, Rotation2d.fromRotations(-0.000732));
            }
        };
    }

    // #endregion

    // #region: ----- Traverser -----
    @Override
    public double getTraverserFowardVelocity() {
        throw new UnsupportedOperationException("Unimplemented method 'getTraverserFowardVoltage'");
    }

    @Override
    public double getTraverserReverseVelocity() {
        throw new UnsupportedOperationException("Unimplemented method 'getTraverserReverseVoltage'");
    }

    @Override
    public boolean isTraverserInverted() {
        throw new UnsupportedOperationException("Unimplemented method 'isTraverserInverted'");
    }

    // #endregion

    // #region: ----- Vision -----
    @Override
    public String getCameraName() {
        return "limelight";
    }

    // #endregion

    // #region: --------------- Operation Modes --------------------------------
    @Override
    public boolean isDrivingModeFieldRelative() {
        return false;
    }

    // #endregion

    // #region: --------------- Physical Measurements --------------------------
    @Override
    public double getGearRatioOfDriveWheel() {
        // L2 Gear ratio.
        return (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    }

    @Override
    public double getGearRatioOfTurnWheel() {
        return 12.8;
    }

    @Override
    public Measure<Distance> getWheelDistanceFrontToBack() {
        return Inches.of(24);
    }

    @Override
    public Measure<Distance> getWheelDistanceLeftToRight() {
        return Inches.of(24);
    }

    public Measure<Distance> getWheelRadius() {
        return Inches.of(2);
    }

    // #endregion
}
