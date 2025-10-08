package frc.trigon.robot.subsystems.armelevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import lib.hardware.RobotHardwareStats;
import lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import lib.hardware.phoenix6.cancoder.CANcoderSignal;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import lib.hardware.simulation.ElevatorSimulation;
import lib.hardware.simulation.SingleJointedArmSimulation;
import lib.utilities.mechanisms.ElevatorMechanism2d;
import lib.utilities.mechanisms.SingleJointedArmMechanism2d;

public class ArmElevatorConstants {
    private static final int
            ARM_MASTER_MOTOR_ID = 13,
            ARM_FOLLOWER_MOTOR_ID = 14,
            ANGLE_ENCODER_ID = 13,
            ELEVATOR_MASTER_MOTOR_ID = 16,
            ELEVATOR_FOLLOWER_MOTOR_ID = 17;
    private static final String
            ARM_MASTER_MOTOR_NAME = "ArmMasterMotor",
            ARM_FOLLOWER_MOTOR_NAME = "ArmFollowerMotor",
            ANGLE_ENCODER_NAME = "ArmEncoder",
            ELEVATOR_MASTER_MOTOR_NAME = "ElevatorMasterMotor",
            ELEVATOR_FOLLOWER_MOTOR_NAME = "ElevatorFollowerMotor";
    static final TalonFXMotor
            ARM_MASTER_MOTOR = new TalonFXMotor(ARM_MASTER_MOTOR_ID, ARM_MASTER_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            ARM_FOLLOWER_MOTOR = new TalonFXMotor(ARM_FOLLOWER_MOTOR_ID, ARM_FOLLOWER_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            ELEVATOR_MASTER_MOTOR = new TalonFXMotor(ELEVATOR_MASTER_MOTOR_ID, ELEVATOR_MASTER_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            ELEVATOR_FOLLOWER_MOTOR = new TalonFXMotor(ELEVATOR_FOLLOWER_MOTOR_ID, ELEVATOR_FOLLOWER_MOTOR_NAME, RobotConstants.CANIVORE_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ANGLE_ENCODER_NAME, RobotConstants.CANIVORE_NAME);

    static final double
            ARM_GEAR_RATIO = 55.273,
            ELEVATOR_GEAR_RATIO = 4;
    private static final double
            ARM_MOTOR_CURRENT_LIMIT = 50,
            ELEVATOR_MOTOR_CURRENT_LIMIT = 50;
    private static final double ANGLE_ENCODER_GRAVITY_OFFSET = 0.184814453125 - 0.25;
    static final double ARM_POSITION_OFFSET_FROM_GRAVITY_OFFSET = RobotHardwareStats.isSimulation() ? 0 : 0.184814453125 - 0.25 - ANGLE_ENCODER_GRAVITY_OFFSET;
    private static final boolean
            SHOULD_ARM_FOLLOWER_OPPOSE_MASTER = false,
            SHOULD_ELEVATOR_FOLLOWER_OPPOSE_MASTER = false;
    static final double
            ARM_DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 2.4614 : 2,
            ARM_DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 67.2344 : 3.5,
            ARM_DEFAULT_MAXIMUM_JERK = ARM_DEFAULT_MAXIMUM_ACCELERATION * 10,
            ELEVATOR_DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 25.178 : 21,
            ELEVATOR_DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 80 : 55;
    static final boolean FOC_ENABLED = true;

    public static final double
            ARM_LENGTH_METERS = 0.52,
            ELEVATOR_MASS_KILOGRAMS = 7,
            DRUM_RADIUS_METERS = 0.04,
            MINIMUM_ELEVATOR_HEIGHT_METERS = 0,
            MAXIMUM_ELEVATOR_HEIGHT_METERS = 1.644;
    private static final int
            ARM_MOTOR_AMOUNT = 2,
            ELEVATOR_MOTOR_AMOUNT = 2;
    private static final DCMotor
            ARM_GEARBOX = DCMotor.getKrakenX60Foc(ARM_MOTOR_AMOUNT),
            ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(ELEVATOR_MOTOR_AMOUNT);
    private static final double
            ARM_MASS_KILOGRAMS = 3.5;
    private static final Rotation2d
            ARM_MINIMUM_ANGLE = Rotation2d.fromDegrees(-90),
            ARM_MAXIMUM_ANGLE = Rotation2d.fromDegrees(270);
    private static final boolean SHOULD_ARM_SIMULATE_GRAVITY = true;
    private static final boolean SHOULD_ELEVATOR_SIMULATE_GRAVITY = true;
    private static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            ARM_GEARBOX,
            ARM_GEAR_RATIO,
            ARM_LENGTH_METERS,
            ARM_MASS_KILOGRAMS,
            ARM_MINIMUM_ANGLE,
            ARM_MAXIMUM_ANGLE,
            SHOULD_ARM_SIMULATE_GRAVITY
    );

    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(
            ELEVATOR_GEARBOX,
            ELEVATOR_GEAR_RATIO,
            ELEVATOR_MASS_KILOGRAMS,
            DRUM_RADIUS_METERS,
            MINIMUM_ELEVATOR_HEIGHT_METERS,
            MAXIMUM_ELEVATOR_HEIGHT_METERS,
            SHOULD_ELEVATOR_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config ARM_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Seconds),
            Units.Volts.of(1.5),
            Units.Second.of(1000)
    );

    static final SysIdRoutine.Config ELEVATOR_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.25).per(Units.Seconds),
            Units.Volts.of(2),
            Units.Second.of(1000)
    );

    static final SingleJointedArmMechanism2d ARM_MECHANISM = new SingleJointedArmMechanism2d(
            "ArmMechanism",
            ARM_LENGTH_METERS,
            Color.kBlue
    );

    static final ElevatorMechanism2d ELEVATOR_MECHANISM = new ElevatorMechanism2d(
            "ElevatorMechanism",
            MAXIMUM_ELEVATOR_HEIGHT_METERS + 0.1,
            MINIMUM_ELEVATOR_HEIGHT_METERS,
            Color.kYellow
    );

    static final Pose3d ARM_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, -0.0954, 0.3573),
            new Rotation3d(0, 0, 0)
    );

    public static final Pose3d ELEVATOR_FIRST_STAGE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, -0.17, 0.0504),
            new Rotation3d(0, 0, 0)
    );

    static final Pose3d ELEVATOR_SECOND_STAGE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, -0.17, 0.095),
            new Rotation3d(0, 0, 0)
    );

    static final Transform3d ARM_TO_HELD_GAME_PIECE = new Transform3d(
            new Translation3d(0, 0.1, -0.5855),
            new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(0), 0)
    );
    static final double SECOND_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS = 0.593;
    static final double DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.2);

    /**
     * The highest point of the arms angular zone where the safety logic applies.
     */
    static final Rotation2d MAXIMUM_ARM_SAFE_ANGLE = Rotation2d.fromDegrees(0);

    /**
     * The lowest point in the Elevators zone where the safety logic applies.
     */
    public static final double MINIMUM_ELEVATOR_SAFE_ZONE_METERS = 0.05;
    static final double ELEVATOR_POSITION_TOLERANCE_METERS = 0.02;

    static final Trigger IS_BREAKING_ARM_EVENT = new Trigger(
            () -> Math.abs(ARM_MASTER_MOTOR.getSignal(TalonFXSignal.STATOR_CURRENT)) > 70
    ).debounce(0.1);

    static {
        configureArmMasterMotor();
        configureArmFollowerMotor();
        configureElevatorMasterMotor();
        configureElevatorFollowerMotor();
        configureAngleEncoder();
        IS_BREAKING_ARM_EVENT.onTrue(new InstantCommand(ARM_MASTER_MOTOR::stopMotor));
    }

    private static void configureArmMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = ARM_GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ANGLE_ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 34 : 60;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 3 : 1.2;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.026331 : 0.06;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 4.8752 : 5.6;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.17848 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.1117 : 0.358;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

//        config.Feedback.VelocityFilterTimeConstant = 0.2;

        config.MotionMagic.MotionMagicCruiseVelocity = ARM_DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ARM_DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

//        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.degreesToRotations(270);

//        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.degreesToRotations(-270);

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ARM_MOTOR_CURRENT_LIMIT;

        ARM_MASTER_MOTOR.applyConfiguration(config);
        ARM_MASTER_MOTOR.setPhysicsSimulation(ARM_SIMULATION);

        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
    }

    private static void configureArmFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ARM_MOTOR_CURRENT_LIMIT;

        ARM_FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(ARM_MASTER_MOTOR.getID(), SHOULD_ARM_FOLLOWER_OPPOSE_MASTER);
        ARM_FOLLOWER_MOTOR.setControl(followerRequest);
    }

    private static void configureElevatorMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 3.5 : 12;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.4 : 1;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.016165 : 0.0546875;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.4766 : 0.4;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.014239 : 0;//0.036238
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.58202 : 0.41015625;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

//        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ELEVATOR_REVERSE_LIMIT_RESET_POSITION_ROTATIONS;
//
//        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.8;

        config.MotionMagic.MotionMagicCruiseVelocity = ELEVATOR_DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ELEVATOR_DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ELEVATOR_MOTOR_CURRENT_LIMIT;

        ELEVATOR_MASTER_MOTOR.applyConfiguration(config);
        ELEVATOR_MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        ELEVATOR_MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ELEVATOR_MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ELEVATOR_MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ELEVATOR_MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        ELEVATOR_MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ELEVATOR_MASTER_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
//        ELEVATOR_MASTER_MOTOR.setPosition(0);
    }

    private static void configureElevatorFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ELEVATOR_MOTOR_CURRENT_LIMIT;

        ELEVATOR_FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(ELEVATOR_MASTER_MOTOR.getID(), SHOULD_ELEVATOR_FOLLOWER_OPPOSE_MASTER);
        ELEVATOR_FOLLOWER_MOTOR.setControl(followerRequest);
    }

    private static void configureAngleEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(ARM_MASTER_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    public enum ArmElevatorState {
        PREPARE_SCORE_L1(Rotation2d.fromDegrees(-8), 0.5, null, false, 1),
        PREPARE_SCORE_L2(Rotation2d.fromDegrees(60), 0.13, null, false, 1),
        PREPARE_SCORE_L3(Rotation2d.fromDegrees(60), 0.6, null, false, 1),
        PREPARE_SCORE_L4(Rotation2d.fromDegrees(50), 1.5, null, false, 1),
        PREPARE_REST(Rotation2d.fromDegrees(-90), 0.603, null, false, 0.8),
        REST(Rotation2d.fromDegrees(-90), 0.534, PREPARE_REST, true, 0.8),
        REST_AFTER_LOADING(Rotation2d.fromDegrees(-90), 0.603, null, true, 0.7),
        REST_WITH_CORAL(Rotation2d.fromDegrees(90), 0.603, null, false, 0.8),
        REST_WITH_ALGAE(Rotation2d.fromDegrees(90), 0.603, null, false, 0.3),
        PREPARE_FOR_REST_FOR_CLIMB(Rotation2d.fromDegrees(-29), 0.6, null, false, 0.7),
        REST_FOR_CLIMB(Rotation2d.fromDegrees(-29), 0, PREPARE_FOR_REST_FOR_CLIMB, true, 0.7),
        ZERO_ELEVATOR(Rotation2d.fromDegrees(90), 0.65, null, false, 0.7),
        LOAD_CORAL(Rotation2d.fromDegrees(-90), 0.485, null, true, 0.7),
        UNLOAD_CORAL(Rotation2d.fromDegrees(-90), 0.603, null, false, 0.7),
        EJECT(Rotation2d.fromDegrees(-30), 0.603, null, false, 0.7),
        SCORE_L1(Rotation2d.fromDegrees(-13), PREPARE_SCORE_L1.targetPositionMeters, null, false, 1),
        SCORE_L2(Rotation2d.fromDegrees(25), PREPARE_SCORE_L2.targetPositionMeters, PREPARE_SCORE_L2, false, 0.8),
        SCORE_L3(Rotation2d.fromDegrees(25), PREPARE_SCORE_L3.targetPositionMeters, PREPARE_SCORE_L3, false, 0.8),
        SCORE_L4(Rotation2d.fromDegrees(-10), 1.5, PREPARE_SCORE_L4, false, 0.8),
        SCORE_NET(Rotation2d.fromDegrees(70), 1.61, null, false, 0.3),
        SCORE_PROCESSOR(Rotation2d.fromDegrees(0), 0.603, null, false, 0.7),
        COLLECT_ALGAE_L2(Rotation2d.fromDegrees(0), 0.603, null, false, 1),
        COLLECT_ALGAE_L3(Rotation2d.fromDegrees(0), 1.2, null, false, 1),
        PREPARE_COLLECT_ALGAE_FLOOR(Rotation2d.fromDegrees(-30), 0.3, null, false, 1),
        COLLECT_ALGAE_FLOOR(Rotation2d.fromDegrees(-30), 0.13, PREPARE_COLLECT_ALGAE_FLOOR, true, 1),
        COLLECT_ALGAE_LOLLIPOP(Rotation2d.fromDegrees(0), 0.15, null, false, 1);


        public final Rotation2d targetAngle;
        public final double targetPositionMeters;
        public final ArmElevatorState prepareState;
        public final boolean ignoreConstraints;
        final double speedScalar;

        ArmElevatorState(Rotation2d targetAngle, double targetPositionMeters, ArmElevatorState prepareState, boolean ignoreConstraints, double speedScalar) {
            this.targetAngle = targetAngle;
            this.targetPositionMeters = targetPositionMeters;
            this.prepareState = prepareState;
            this.ignoreConstraints = ignoreConstraints;
            this.speedScalar = speedScalar;
        }
    }
}