package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;
import lib.hardware.misc.leds.LEDCommands;

public class CoralCollectionCommands {
    public static Command getCoralCollectionCommand() {
        return new ParallelCommandGroup(
                getIntakeSequenceCommand()
               // new IntakeAssistCommand(OperatorConstants.DEFAULT_INTAKE_ASSIST_MODE)
        );
    }

    private static Command getIntakeSequenceCommand() {
        return new SequentialCommandGroup(
                getInitiateCollectionCommand().until(RobotContainer.INTAKE::hasCoral),
                new InstantCommand(() -> getAlignCoralCommand().schedule()).alongWith(getCollectionConfirmationCommand())
        );
    }

    private static Command getInitiateCollectionCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.COLLECT)
        );
    }

    private static Command getAlignCoralCommand() {
        return new SequentialCommandGroup(
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.ALIGN_CORAL).until(RobotContainer.TRANSPORTER::hasCoral),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.HOLD_CORAL)
        );
    }

    private static Command getCollectionConfirmationCommand() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(OperatorConstants.RUMBLE_DURATION_SECONDS, OperatorConstants.RUMBLE_POWER)),
                LEDCommands.getAnimateCommand(LEDConstants.COLLECTION_CONFIRMATION_ANIMATION_SETTINGS) //TODO: add LEDs
        );
    }
}