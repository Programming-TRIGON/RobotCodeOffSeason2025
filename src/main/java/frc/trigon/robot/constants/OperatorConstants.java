package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.commands.commandfactories.AlgaeManipulationCommands;
import frc.trigon.robot.misc.ReefChooser;
import lib.hardware.misc.KeyboardController;
import lib.hardware.misc.XboxController;

public class OperatorConstants {
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.07;
    public static final double
            RUMBLE_DURATION_SECONDS = 0.7,
            RUMBLE_POWER = 1;
    private static final int
            DRIVER_CONTROLLER_PORT = 0,
            REEF_CHOOSER_PORT = 1;
    private static final int
            DRIVER_CONTROLLER_RIGHT_STICK_EXPONENT = 1,
            DRIVER_CONTROLLER_LEFT_STICK_EXPONENT = 2;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, DRIVER_CONTROLLER_RIGHT_STICK_EXPONENT, DRIVER_CONTROLLER_LEFT_STICK_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();
    public static final ReefChooser REEF_CHOOSER = new ReefChooser(REEF_CHOOSER_PORT);
    public static boolean SHOULD_FLIP_ARM_OVERRIDE = false;//TODO: Implement everywhere
    private static boolean
            IS_LEFT_SCORE_BUTTON_PRESSED = false,
            IS_RIGHT_SCORE_BUTTON_PRESSED = false;

    public static final double
            POV_DIVIDER = 2,
            TRANSLATION_STICK_SPEED_DIVIDER = 1,
            ROTATION_STICK_SPEED_DIVIDER = 1;

    public static final double INTAKE_ASSIST_SCALAR = 0.0;
    public static final IntakeAssistCommand.AssistMode DEFAULT_INTAKE_ASSIST_MODE = IntakeAssistCommand.AssistMode.ALTERNATE_ASSIST;

    public static final Trigger
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.povUp(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVER_CONTROLLER.getPov() != -1),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            DEBUGGING_TRIGGER = OPERATOR_CONTROLLER.f2(),
            CONTINUE_TRIGGER = DRIVER_CONTROLLER.leftStick().and(DRIVER_CONTROLLER.rightStick()).or(OPERATOR_CONTROLLER.k()),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down();
    public static final Trigger
            FLOOR_ALGAE_COLLECTION_TRIGGER = DRIVER_CONTROLLER.leftBumper(),//TODO: Add operator control
            REEF_ALGAE_COLLECTION_TRIGGER = DRIVER_CONTROLLER.rightBumper().or(OPERATOR_CONTROLLER.a()),
            STOP_REEF_ALGAE_ALIGN_TRIGGER = DRIVER_CONTROLLER.x().and(RobotContainer.ARM::hasGamePiece),
            SCORE_ALGAE_IN_NET_TRIGGER = OPERATOR_CONTROLLER.n().or(createScoreTrigger(true, true)),
            SCORE_ALGAE_IN_PROCESSOR_TRIGGER = OPERATOR_CONTROLLER.j().or(createScoreTrigger(false, true)),
            CORAL_COLLECTION_TRIGGER = DRIVER_CONTROLLER.leftTrigger().or(OPERATOR_CONTROLLER.c()),
            SCORE_CORAL_RIGHT_TRIGGER = createScoreTrigger(true, false),
            SCORE_CORAL_LEFT_TRIGGER = createScoreTrigger(false, false),
            EJECT_CORAL_TRIGGER = OPERATOR_CONTROLLER.e();
    public static final Trigger
            SPAWN_CORAL_IN_SIMULATION_TRIGGER = OPERATOR_CONTROLLER.equals(),
            FLIP_ARM_TRIGGER = DRIVER_CONTROLLER.start(),
            LOLLIPOP_ALGAE_TOGGLE_TRIGGER = DRIVER_CONTROLLER.a();

    public static final Trigger
            SET_TARGET_SCORING_REEF_LEVEL_L1_TRIGGER = OPERATOR_CONTROLLER.numpad0().or(DRIVER_CONTROLLER.a()),
            SET_TARGET_SCORING_REEF_LEVEL_L2_TRIGGER = OPERATOR_CONTROLLER.numpad1().or(DRIVER_CONTROLLER.b()),
            SET_TARGET_SCORING_REEF_LEVEL_L3_TRIGGER = OPERATOR_CONTROLLER.numpad2().or(DRIVER_CONTROLLER.x()),
            SET_TARGET_SCORING_REEF_LEVEL_L4_TRIGGER = OPERATOR_CONTROLLER.numpad3().or(DRIVER_CONTROLLER.y()),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_2_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad9(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_4_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad6(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_6_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad5(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_8_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad4(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_10_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad7(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_12_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad8(),
            SET_TARGET_REEF_SCORING_SIDE_LEFT_TRIGGER = OPERATOR_CONTROLLER.left(),
            SET_TARGET_REEF_SCORING_SIDE_RIGHT_TRIGGER = OPERATOR_CONTROLLER.right();

    private static Trigger createScoreTrigger(boolean isRight, boolean isAlgaeCommand) {
        final Trigger scoreTrigger;

        if (isRight)
            scoreTrigger = DRIVER_CONTROLLER.rightStick()
                    .and(() -> !IS_LEFT_SCORE_BUTTON_PRESSED)
                    .onTrue(new InstantCommand(() -> IS_RIGHT_SCORE_BUTTON_PRESSED = true))
                    .onFalse(new InstantCommand(() -> IS_RIGHT_SCORE_BUTTON_PRESSED = false));
        else
            scoreTrigger = DRIVER_CONTROLLER.leftStick()
                    .and(() -> !IS_RIGHT_SCORE_BUTTON_PRESSED)
                    .onTrue(new InstantCommand(() -> IS_LEFT_SCORE_BUTTON_PRESSED = true))
                    .onFalse(new InstantCommand(() -> IS_LEFT_SCORE_BUTTON_PRESSED = false));

        if (isAlgaeCommand)
            return scoreTrigger.and(() -> AlgaeManipulationCommands.isHoldingAlgae());
        return scoreTrigger.and(() -> !AlgaeManipulationCommands.isHoldingAlgae());
    }
}