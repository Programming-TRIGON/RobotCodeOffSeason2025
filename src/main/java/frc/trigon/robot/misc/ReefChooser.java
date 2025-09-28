package frc.trigon.robot.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.trigon.robot.commands.commandfactories.CoralPlacingCommands;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.ArmElevatorConstants;

import java.util.function.Supplier;

public class ReefChooser {
    private final CommandGenericHID hid;
    private CoralPlacingCommands.ScoringLevel scoringLevel = CoralPlacingCommands.ScoringLevel.L4;
    private FieldConstants.ReefSide reefSide = FieldConstants.ReefSide.LEFT;

    public ReefChooser(int port) {
        hid = new CommandGenericHID(port);

        new WaitCommand(3).andThen(this::configureBindings).ignoringDisable(true).schedule();
    }

    public CoralPlacingCommands.ScoringLevel getScoringLevel() {
        return scoringLevel;
    }


    public ArmElevatorConstants.ArmElevatorState getArmElevatorState() {
        return scoringLevel.armElevatorState;
    }

    private void configureBindings() {
        configureScoringLevelBindings();
        configureReefHIDBindings();
        configureFallbackBindings();
    }

    private void configureScoringLevelBindings() {
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L1_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L1));
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L2_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L2));
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L3_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L3));
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L4_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L4));
    }

    private Command getSetTargetLevelCommand(Supplier<CoralPlacingCommands.ScoringLevel> scoringLevel) {
        return new InstantCommand(
                () -> this.scoringLevel = scoringLevel.get()
        ).ignoringDisable(true);
    }

    private void configureReefHIDBindings() {
        for (int i = 0; i < 12; i++)
            hid.button(i + 1).onTrue(getSetFaceFromIndexCommand(i));
    }

    private Command getSetFaceFromIndexCommand(int index) {
        return new InstantCommand(
                () -> setFaceSideFromIndex(index)
        ).ignoringDisable(true);
    }

    private void setFaceSideFromIndex(int index) {
        reefSide = FieldConstants.ReefSide.values()[index % 2];
    }

    private void configureFallbackBindings() {
        OperatorConstants.SET_TARGET_REEF_SCORING_SIDE_LEFT_TRIGGER.onTrue(getSetTargetReefSideCommand(FieldConstants.ReefSide.LEFT));
        OperatorConstants.SET_TARGET_REEF_SCORING_SIDE_RIGHT_TRIGGER.onTrue(getSetTargetReefSideCommand(FieldConstants.ReefSide.RIGHT));
    }

    private Command getSetTargetReefSideCommand(FieldConstants.ReefSide reefSide) {
        return new InstantCommand(
                () -> this.reefSide = reefSide
        ).ignoringDisable(true);
    }
}