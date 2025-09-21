package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import lib.hardware.RobotHardwareStats;
import lib.hardware.misc.simplesensor.SimpleSensor;
import lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import lib.hardware.phoenix6.cancoder.CANcoderSignal;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import lib.hardware.simulation.SimpleMotorSimulation;
import lib.hardware.simulation.SingleJointedArmSimulation;
import lib.utilities.Conversions;
import lib.utilities.mechanisms.SingleJointedArmMechanism2d;
import lib.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class ArmConstants {
    private static final int
            ARM_MASTER_MOTOR_ID = 13,
            ARM_FOLLOWER_MOTOR_ID = 14,
            END_EFFECTOR_MOTOR_ID = 15,
            ANGLE_ENCODER_ID = 13,
            DISTANCE_SENSOR_CHANNEL = 3;
    private static final String
            ARM_MASTER_MOTOR_NAME = "ArmMasterMotor",
            ARM_FOLLOWER_MOTOR_NAME = "ArmFollowerMotor",
            END_EFFECTOR_MOTOR_NAME = "EndEffectorMotor",
            ANGLE_ENCODER_NAME = "ArmEncoder",
            DISTANCE_SENSOR_NAME = "EndEffectorDistanceSensor";
    static final TalonFXMotor
            ARM_MASTER_MOTOR = new TalonFXMotor(ARM_MASTER_MOTOR_ID, ARM_MASTER_MOTOR_NAME),
            ARM_FOLLOWER_MOTOR = new TalonFXMotor(ARM_FOLLOWER_MOTOR_ID, ARM_FOLLOWER_MOTOR_NAME),
            END_EFFECTOR_MOTOR = new TalonFXMotor(END_EFFECTOR_MOTOR_ID, END_EFFECTOR_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ANGLE_ENCODER_NAME);
    static final SimpleSensor DISTANCE_SENSOR = SimpleSensor.createDigitalSensor(DISTANCE_SENSOR_CHANNEL, DISTANCE_SENSOR_NAME);

    private static final double
            ARM_GEAR_RATIO = 40,
            END_EFFECTOR_GEAR_RATIO = 17;
    private static final double ARM_MOTOR_CURRENT_LIMIT = 50;
    private static final double ANGLE_ENCODER_GRAVITY_OFFSET = 0;
    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET = RobotHardwareStats.isSimulation() ? 0 - Conversions.degreesToRotations(90) : 0 + Conversions.degreesToRotations(0) - ANGLE_ENCODER_GRAVITY_OFFSET;
    private static final boolean SHOULD_ARM_FOLLOWER_OPPOSE_MASTER = false;
    static final double
            ARM_DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 2.4614 : 0,
            ARM_DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 67.2344 : 0,
            ARM_DEFAULT_MAXIMUM_JERK = ARM_DEFAULT_MAXIMUM_ACCELERATION * 10;
    static final boolean FOC_ENABLED = true;

    public static final double ARM_LENGTH_METERS = 0.52;
    private static final int
            ARM_MOTOR_AMOUNT = 2,
            END_EFFECTOR_MOTOR_AMOUNT = 1;
    private static final DCMotor
            ARM_GEARBOX = DCMotor.getKrakenX60Foc(ARM_MOTOR_AMOUNT),
            END_EFFECTOR_GEARBOX = DCMotor.getKrakenX60Foc(END_EFFECTOR_MOTOR_AMOUNT);
    private static final double
            ARM_MASS_KILOGRAMS = 3.5,
            END_EFFECTOR_MOMENT_OF_INERTIA = 0.003,
            END_EFFECTOR_MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    private static final Rotation2d
            ARM_MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            ARM_MAXIMUM_ANGLE = Rotation2d.fromDegrees(360);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            ARM_GEARBOX,
            ARM_GEAR_RATIO,
            ARM_LENGTH_METERS,
            ARM_MASS_KILOGRAMS,
            ARM_MINIMUM_ANGLE,
            ARM_MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );
    private static final SimpleMotorSimulation END_EFFECTOR_SIMULATION = new SimpleMotorSimulation(
            END_EFFECTOR_GEARBOX,
            END_EFFECTOR_GEAR_RATIO,
            END_EFFECTOR_MOMENT_OF_INERTIA
    );
    private static final DoubleSupplier DISTANCE_SENSOR_SIMULATION_SUPPLIER = () -> SimulationFieldHandler.isHoldingGamePiece() ? 0 : 1;

    static final SysIdRoutine.Config ARM_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    static final SingleJointedArmMechanism2d ARM_MECHANISM = new SingleJointedArmMechanism2d(
            "ArmMechanism",
            ARM_LENGTH_METERS,
            Color.kBlue
    );
    static final SpeedMechanism2d END_EFFECTOR_MECHANISM = new SpeedMechanism2d(
            "EndEffectorMechanism",
            END_EFFECTOR_MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static final Pose3d ARM_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, -0.0954, 0.3573),
            new Rotation3d(0, 0, 0)
    );
    /**
     * The highest point of the arms angular zone where the safety logic applies.
     */
    static final Rotation2d MAXIMUM_ARM_SAFE_ANGLE = Rotation2d.fromDegrees(90);
    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.5);
    private static final double COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS = 0.2;
    static final BooleanEvent COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            DISTANCE_SENSOR::getBinaryValue
    ).debounce(COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS);

    static {
        configureArmMasterMotor();
        configureArmFollowerMotor();
        configureEndEffectorMotor();
        configureAngleEncoder();
        configureDistanceSensor();
    }

    private static void configureArmMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = ARM_GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ARM_MASTER_MOTOR.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 38 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 1 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.026331 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 4.8752 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.17848 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.1117 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = ARM_DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ARM_DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ARM_MOTOR_CURRENT_LIMIT;

        ARM_MASTER_MOTOR.applyConfiguration(config);
        ARM_MASTER_MOTOR.setPhysicsSimulation(ARM_SIMULATION);

        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        ARM_MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
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

    private static void configureEndEffectorMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.RotorToSensorRatio = END_EFFECTOR_GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;

        END_EFFECTOR_MOTOR.applyConfiguration(config);
        END_EFFECTOR_MOTOR.setPhysicsSimulation(END_EFFECTOR_SIMULATION);

        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureAngleEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(ARM_MASTER_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    private static void configureDistanceSensor() {
        DISTANCE_SENSOR.setSimulationSupplier(DISTANCE_SENSOR_SIMULATION_SUPPLIER);
    }

    public enum ArmState {
        REST(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        REST_WITH_CORAL(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), 0),
        REST_FOR_CLIMB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        LOAD_CORAL(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), -4),
        HOLD_ALGAE(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), -4),
        EJECT(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(60), 4),
        SCORE_L1(Rotation2d.fromDegrees(110), Rotation2d.fromDegrees(110), 4),
        SCORE_L2(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(100), 4),
        SCORE_L3(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(100), 4),
        SCORE_L4(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(100), 4),
        SCORE_NET(Rotation2d.fromDegrees(160), Rotation2d.fromDegrees(160), 4),
        SCORE_PROCESSOR(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), 4),
        COLLECT_ALGAE_L2(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), -4),
        COLLECT_ALGAE_L3(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), -4);

        public final Rotation2d targetAngle;
        public final Rotation2d prepareAngle;
        public final double targetEndEffectorVoltage;

        ArmState(Rotation2d targetAngle, Rotation2d prepareAngle, double targetEndEffectorVoltage) {
            this.targetAngle = targetAngle;
            this.prepareAngle = prepareAngle;
            this.targetEndEffectorVoltage = targetEndEffectorVoltage;
        }
    }
}