package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.armelevator.ArmElevatorCommands;
import frc.trigon.robot.subsystems.armelevator.ArmElevatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A class that contains the general commands of the robot, such as commands that alter a command or commands that affect all subsystems.
 * These are different from {@link CommandConstants} because they create new commands that use some form of logic instead of only constructing an existing command with parameters.
 */
public class GeneralCommands {
    public static Command getFieldRelativeDriveCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getRightX())
        );
    }

    public static Command getToggleBrakeCommand() {
        return new InstantCommand(() -> {
            MotorSubsystem.IS_BRAKING = !MotorSubsystem.IS_BRAKING;
            MotorSubsystem.setAllSubsystemsBrakeAsync(MotorSubsystem.IS_BRAKING);
        }).ignoringDisable(true);
    }

    public static Command getToggleShouldLoadCoralCommand() {
        return new InstantCommand(() -> {
            CoralCollectionCommands.SHOULD_LOAD_CORAL = !CoralCollectionCommands.SHOULD_LOAD_CORAL;
        }
        );
    }

    public static Command getToggleShouldManipulateCoralAtonomouslyCommand() {
        return new InstantCommand(() -> {
            if (CoralCollectionCommands.SHOULD_USE_INTAKE_ASSIST || CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY) {
                CoralCollectionCommands.SHOULD_USE_INTAKE_ASSIST = false;
                CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY = false;
            } else {
                CoralCollectionCommands.SHOULD_USE_INTAKE_ASSIST = true;
                CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY = true;
            }
        }
        );
    }

    public static Command getToggleShouldCollectCoralAtonomouslyCommand() {
        return new InstantCommand(() -> {
            CoralCollectionCommands.SHOULD_USE_INTAKE_ASSIST = !CoralCollectionCommands.SHOULD_USE_INTAKE_ASSIST;
        }
        );
    }

    public static Command getToggleShouldScoreCoralAtonomouslyCommand() {
        return new InstantCommand(() -> {
            CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY = !CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY;
        }
        );
    }

    public static Command getDelayedCommand(double delaySeconds, Runnable toRun) {
        return new WaitCommand(delaySeconds).andThen(toRun).ignoringDisable(true);
    }

    public static Command getContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        return new ConditionalCommand(
                onTrue.onlyWhile(condition),
                onFalse.until(condition),
                condition
        ).repeatedly();
    }

    /**
     * A command that only runs when a condition is met.
     *
     * @param command   the command to run
     * @param condition the condition that needs to be met for the command to run
     * @return the command
     */
    public static Command runWhen(Command command, BooleanSupplier condition) {
        return new WaitUntilCommand(condition).andThen(command);
    }

    /**
     * <B><font color="red">---- UNTESTED ----</font></B> <br>
     * A command that only runs when a condition is met for a certain amount of time.
     *
     * @param command             the command to run
     * @param condition           the condition that needs to be met for the command to run
     * @param debounceTimeSeconds the time that the condition needs to be true for the command to run
     * @return the command
     */
    public static Command runWhen(Command command, BooleanSupplier condition, double debounceTimeSeconds) {
        final Debouncer debouncer = new Debouncer(0, Debouncer.DebounceType.kRising);
        return new SequentialCommandGroup(
                new InstantCommand(() -> debouncer.setDebounceTime(debounceTimeSeconds)),
                runWhen(command, () -> debouncer.calculate(condition.getAsBoolean()))
        );
    }

    public static Command getResetFlipArmOverrideCommand() {
        return new InstantCommand(() -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE = false);
    }

    public static Command getFlippableOverridableArmCommand(ArmElevatorConstants.ArmElevatorState targetState, boolean isPrepareState, BooleanSupplier shouldStartFlipped) {
        return isPrepareState ?
                ArmElevatorCommands.getPrepareForStateCommand(() -> targetState, () -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE ^ shouldStartFlipped.getAsBoolean()) :
                ArmElevatorCommands.getSetTargetStateCommand(() -> targetState, () -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE ^ shouldStartFlipped.getAsBoolean());
    }

    public static Command getFlippableOverridableArmCommand(Supplier<ArmElevatorConstants.ArmElevatorState> targetState, boolean isPrepareState, BooleanSupplier shouldStartFlipped) {
        return isPrepareState ?
                ArmElevatorCommands.getPrepareForStateCommand(targetState, () -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE ^ shouldStartFlipped.getAsBoolean()) :
                ArmElevatorCommands.getSetTargetStateCommand(targetState, () -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE ^ shouldStartFlipped.getAsBoolean());
    }

    public static Command getFlippableOverridableArmCommand(ArmElevatorConstants.ArmElevatorState targetState, boolean isPrepareState) {
        return isPrepareState ?
                ArmElevatorCommands.getPrepareForStateCommand(() -> targetState, () -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE) :
                ArmElevatorCommands.getSetTargetStateCommand(() -> targetState, () -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE);
    }
}