package frc.robot.commands;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.NoteSensor;

public class ShooterCommands {
    public static class IntakeCommand extends Command {
        private final Intake intake;
        private final Feeder feeder;

        public IntakeCommand(Intake intake, Feeder feeder) {
            this.intake = intake;
            this.feeder = feeder;
            addRequirements(intake, feeder);
        }

        @Override
        public void initialize() {
            intake.start();
            feeder.start();
        }

        @Override
        public void end(boolean interrupted) {
            intake.stop();
            feeder.stop();
        }
    }

    /** Makes Class non-instantiable */
    private ShooterCommands() {
    }

    // ========================= Default Commands =========================
    public static Command defaultIntakeCommand(Intake intake, NoteSensor sensor) {
        return Commands.run(() -> {
            if (sensor.isObjectDetected()) {
                intake.stop();
            } else {
                intake.start();
            }
        }, intake);
    }

    public static Command defaultFeederCommand(Feeder feeder, NoteSensor sensor) {
        return Commands.run(() -> {
            if (sensor.isObjectDetected()) {
                feeder.stop();
            } else {
                feeder.start();
            }
        }, feeder);
    }

    public static Command defaultFlywheelCommand(Flywheel flywheel) {
        return new SequentialCommandGroup(new WaitCommand(.25), flywheel.stopCommand());
    }

    // ========================= Other Commands =========================

    public static Command reverseShooterCommand(Flywheel flywheel, Feeder feeder, Leds leds) {
        Command reverseShooterCommand = new Command() {
            @Override
            public void execute() {
                flywheel.reverse();
                feeder.reverse();
                leds.setDynamicPattern(new Color[] { Color.kRed, Color.kRed, Color.kBlack, Color.kBlack }, true);
            }

            @Override
            public void end(boolean interrupted) {
                flywheel.stop();
                feeder.stop();
                leds.setAllianceColor();
            }
        };
        reverseShooterCommand.addRequirements(flywheel, feeder, leds);
        return reverseShooterCommand;
    }

    public static Command shootAutonomousCommand(Feeder feeder, NoteSensor noteSensor, Intake intake) {

        ParallelRaceGroup group = new ParallelRaceGroup(
                new StartEndCommand(feeder::start, feeder::stop, feeder),
                new StartEndCommand(intake::start, intake::stop, intake),
                // TODO: May want to wait a little after the note is no longer sensed.
                noteSensor.waitForNoObjectCommandSwitch(),
                new WaitCommand(5));

        return group;
    }

    public static Command shootTeleopCommand(Feeder feeder, Flywheel flywheel, Intake intake, NoteSensor noteSensor) {

        // TODO: Have this run until the Co-Pilot stops pushing the button.
        ParallelRaceGroup group = new ParallelRaceGroup(
                new StartEndCommand(intake::start, intake::stop, intake),
                new StartEndCommand(() -> feeder.setVelocity(11000), feeder::stop,
                        feeder),
                noteSensor.waitForNoObjectCommandSwitch(),
                new WaitCommand(5));

        // TODO: Spin up flywheelsm if not already spinning.
        return group;
    }

    public static Command spinUpFlywheelCommand(Flywheel flywheel) {
        return new SequentialCommandGroup(
                flywheel.startCommand(),
                new WaitCommand(1) // TODO: Tune.
        );
    }

    public static Command stopIntakeFeederCommand(Intake intake, Feeder feeder, Leds leds) {
        return new InstantCommand(() -> {
            intake.stop();
            feeder.stop();
            leds.setDynamicPattern(new Color[] { Color.kRed, Color.kRed, Color.kBlack, Color.kBlack }, true);
        }, intake, feeder);
    }

    public static Command reverseShooterAndIntakeCommand(Intake intake, Feeder feeder, Flywheel flywheel) {
        return new ParallelCommandGroup(new StartEndCommand(flywheel::reverse, flywheel::stop, flywheel),
                new StartEndCommand(feeder::reverse, feeder::stop, feeder),
                new StartEndCommand(intake::reverse, intake::stop, intake));
    }

    public static Command autoAimAtSpeakerCommand(DriveBase driveBase, Aimer aimer) {
        return new ParallelCommandGroup(
                DriveCommands.turnToTargetCommand(driveBase, CONSTANTS::getSpeakerLocation, 4.5), new InstantCommand(
                        () -> aimer.aimAtTarget(CONSTANTS.getSpeakerLocation(), driveBase.getPose().getTranslation())))
                .andThen(new WaitUntilCommand(aimer::atTarget));
    }
}
