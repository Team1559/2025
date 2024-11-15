package frc.robot.subsystems.swerve_module;

import static edu.wpi.first.units.Units.Celsius;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and CANcoder.
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax").
 * </p>
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward motion on the drive motor will propel the robot forward) and
 * copy the reported values from the absolute encoders using AdvantageScope.
 * These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad".
 * </p>
 */
public class SwerveModuleIoTalonFx implements SwerveModuleIo {

    public static TalonFXConfiguration getDefaultTalonFXConfiguration(InvertedValue invertedValue,
            NeutralModeValue breakingMode) {

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 25;
        // currentLimitsConfigs.SupplyCurrentThreshold = 60.0;
        // currentLimitsConfigs.SupplyTimeThreshold = 0.5;
        talonFXConfiguration.CurrentLimits = currentLimitsConfigs;

        MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();
        driveMotorOutputConfigs.Inverted = invertedValue;
        driveMotorOutputConfigs.NeutralMode = breakingMode;
        talonFXConfiguration.withMotorOutput(driveMotorOutputConfigs);

        return talonFXConfiguration;
    }

    private final CANcoder cancoder;
    private final TalonFX driveMotor, steerMotor;

    private final StatusSignal<Double> cancoderAbsolutePosition;
    private final StatusSignal<Double> driveMotorPosition, steerMotorPosition;
    private final StatusSignal<Double> driveMotorVelocity, steerMotorVelocity;
    private final StatusSignal<Double> driveMotorAppliedVolts, steerMotorAppliedVolts;
    private final StatusSignal<Double> driveMotorCurrent, steerMotorCurrent;
    private final StatusSignal<Integer> driveMotorFaults, steerMotorFaults;
    private final StatusSignal<Double> driveMotorTemp, steerMotorTemp;

    private final Rotation2d absoluteEncoderOffset;

    public SwerveModuleIoTalonFx(WheelModuleIndex index) {

        absoluteEncoderOffset = CONSTANTS.getSwerveModuleEncoderOffsets().get(index);

        // ---------- Instantiate Hardware ----------
        cancoder = new CANcoder(CONSTANTS.getSwerveModuleHardwareIds().get(index).CANCODER_ID,
                CONSTANTS.getCanivoreId());
        driveMotor = new TalonFX(CONSTANTS.getSwerveModuleHardwareIds().get(index).DRIVE_MOTOR_ID,
                CONSTANTS.getCanivoreId());
        steerMotor = new TalonFX(CONSTANTS.getSwerveModuleHardwareIds().get(index).STEER_MOTOR_ID,
                CONSTANTS.getCanivoreId());

        // ---------- Configure Hardware ----------
        // ----- Cancoder -----
        // Set CAN Coder configs to defaults.
        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        // ----- Motors -----
        // Only set the Motor Configuration once, to avoid accidentally overriding
        // configs with defaults.
        driveMotor.getConfigurator()
                // Inverted to match our Swerve Drive Module Gear Box & Motors.
                .apply(getDefaultTalonFXConfiguration(InvertedValue.Clockwise_Positive, NeutralModeValue.Brake));
        steerMotor.getConfigurator()
                .apply(getDefaultTalonFXConfiguration(InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake));

        // ---------- Get StatusSignals ----------
        cancoderAbsolutePosition = cancoder.getAbsolutePosition();

        driveMotorPosition = driveMotor.getPosition();
        driveMotorVelocity = driveMotor.getVelocity();
        driveMotorAppliedVolts = driveMotor.getMotorVoltage();
        driveMotorCurrent = driveMotor.getSupplyCurrent();
        driveMotorFaults = driveMotor.getFaultField();
        driveMotorTemp = driveMotor.getDeviceTemp();

        steerMotorPosition = steerMotor.getPosition();
        steerMotorVelocity = steerMotor.getVelocity();
        steerMotorAppliedVolts = steerMotor.getMotorVoltage();
        steerMotorCurrent = steerMotor.getSupplyCurrent();
        steerMotorFaults = steerMotor.getFaultField();
        steerMotorTemp = steerMotor.getDeviceTemp();

        // Set Update frequency.
        BaseStatusSignal.setUpdateFrequencyForAll( // Required for odometry, use faster rate
                CONSTANTS.getPathPlannerLogFrequencyForOdometry(), driveMotorPosition, steerMotorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                CONSTANTS.getPathPlannerLogUpdateFrequencyDefault(),
                driveMotorVelocity,
                driveMotorAppliedVolts,
                driveMotorCurrent,
                cancoderAbsolutePosition,
                steerMotorVelocity,
                steerMotorAppliedVolts,
                steerMotorCurrent);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }

    @Override
    public Measure<Temperature> getMaxSafeMotorTemperature() {
        return CONSTANTS.getFalcon500MaxTemperature();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        steerMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void updateInputs(SwerveModuleIoInputs inputs) {

        BaseStatusSignal.refreshAll(

                cancoderAbsolutePosition,

                driveMotorPosition,
                driveMotorVelocity,
                driveMotorAppliedVolts,
                driveMotorCurrent,
                driveMotorFaults,
                driveMotorTemp,

                steerMotorPosition,
                steerMotorVelocity,
                steerMotorAppliedVolts,
                steerMotorCurrent,
                steerMotorFaults,
                steerMotorTemp);

        inputs.cancoderAbsolutePosition = Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
        inputs.cancoderOffsetPosition = inputs.cancoderAbsolutePosition.minus(absoluteEncoderOffset);

        // Inverted driveMotorPosition so that autonomous sees the robot moving in the
        // correct direction.
        inputs.driveMotorPositionRad = Units.rotationsToRadians(-driveMotorPosition.getValueAsDouble())
                / CONSTANTS.getGearRatioOfDriveWheel();
        inputs.driveMotorVelocityRadPerSec = Units.rotationsToRadians(driveMotorVelocity.getValueAsDouble())
                / CONSTANTS.getGearRatioOfDriveWheel();
        inputs.driveMotorAppliedVolts = driveMotorAppliedVolts.getValueAsDouble();
        inputs.driveMotorCurrentAmps = driveMotorCurrent.getValueAsDouble();
        inputs.driveMotorFaults = driveMotorFaults.getValue();
        inputs.driveMotorTemp = Celsius.of(driveMotorTemp.getValueAsDouble());

        inputs.steerMotorPosition = Rotation2d
                .fromRotations(steerMotorPosition.getValueAsDouble() / CONSTANTS.getGearRatioOfTurnWheel())
                .plus(Rotation2d.fromRadians(0));
        inputs.steerMotorVelocityRadPerSec = Units.rotationsToRadians(steerMotorVelocity.getValueAsDouble())
                / CONSTANTS.getGearRatioOfTurnWheel();
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
        inputs.steerMotorCurrentAmps = steerMotorCurrent.getValueAsDouble();
        inputs.steerMotorFaults = steerMotorFaults.getValue();
        inputs.steerMotorTemp = Celsius.of(steerMotorTemp.getValueAsDouble());
    }
}