package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.arm.ArmConstants;

public class CoralPlacingCommands {
    public enum ScoringLevel {
        L1(),
        L2(),
        L3(),
        L4();

        public final ArmConstants.ArmState armState;
        public final int level = calculateLevel();
        final double xTransformMeters, positiveYTransformMeters;
        final Rotation2d rotationTransform;

        ScoringLevel(double xTransformMeters, double positiveYTransformMeters, Rotation2d rotationTransform) {
            this.xTransformMeters = xTransformMeters;
            this.positiveYTransformMeters = positiveYTransformMeters;
            this.rotationTransform = rotationTransform;
            this.armState = determineArmState;
        }
    }

    private ArmConstants.ArmState determineArmState() {
        return switch (ordinal()) {
            case 0 -> null;
            case 1 -> ArmConstants.ArmState.SCORE_L1;
            case 2 -> ArmConstants.ArmState.SCORE_L2;
            case 3 -> ArmConstants.ArmState.SCORE_L3;
            case 4 -> ArmConstants.ArmState.SCORE_L4;
            default -> throw new IllegalStateException("Unexpected value: " + ordinal());
        };
    }

    private int calculateLevel() {
        if (ordinal() == 0)
            return 1;
        return ordinal();
    }
}
