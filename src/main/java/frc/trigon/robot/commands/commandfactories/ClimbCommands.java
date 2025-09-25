package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.ArmCommands;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.hardware.misc.leds.LEDCommands;

public class ClimbCommands {
    public static boolean IS_CLIMBING = false;//TODO: Make score triggers not work while climbing

    public static Command getClimbCommand() {//TODO: Set other component positions
        return new SequentialCommandGroup(
                new InstantCommand(() -> IS_CLIMBING = true),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.PREPARE_FOR_CLIMB)
                        .until(() -> RobotContainer.CLIMBER.hasCage() || OperatorConstants.CONTINUE_TRIGGER.getAsBoolean()),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB)
                        .until(RobotContainer.CLIMBER::atTargetState),
                getAdjustClimbManuallyCommand()
        ).alongWith(getSetSubsystemsToRestForClimbCommand(), getClimbLEDCommand()).finallyDo(() -> IS_CLIMBING = false);
    }

    private static Command getClimbLEDCommand() {
        return LEDCommands.getAnimateCommand(LEDConstants.CLIMB_ANIMATION_SETTINGS);
    }

    private static Command getAdjustClimbManuallyCommand() {
        return new ParallelCommandGroup(
                ClimberCommands.getSetTargetSpeedCommand(OperatorConstants.DRIVER_CONTROLLER::getRightY),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> 0,
                        () -> 0,
                        () -> 0
                )
        );
    }

    private static Command getSetSubsystemsToRestForClimbCommand() {
        return new ParallelCommandGroup(
                ArmCommands.getSetTargetStateCommand(ArmConstants.ArmState.REST_FOR_CLIMB),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST_FOR_CLIMB),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REST_FOR_CLIMB)
        );
    }
}
