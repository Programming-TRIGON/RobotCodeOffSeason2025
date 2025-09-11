package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import lib.hardware.simulation.SimpleMotorSimulation;
import lib.hardware.simulation.SingleJointedArmSimulation;
import lib.utilities.mechanisms.SingleJointedArmMechanism2d;
import lib.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class IntakeConstants {
    private static final int
            INTAKE_MOTOR_ID = 9,
            ANGLE_MOTOR_ID = 10,
            REVERSE_LIMIT_SENSOR_CHANNEL = 0,
            FORWARD_LIMIT_CHANNEL = 1,
            DISTANCE_SENSOR_CHANNEL = 2;
    private static final String
            INTAKE_MOTOR_NAME = "IntakeMotor",
            ANGLE_MOTOR_NAME = "IntakeAngleMotor",
            REVERSE_LIMIT_SWITCH_NAME = "IntakeReverseLimitSwitch",
            FORWARD_LIMIT_SWITCH_NAME = "IntakeForwardLimitSwitch",
            DISTANCE_SENSOR_NAME = "IntakeDistanceSensor";
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(INTAKE_MOTOR_ID, INTAKE_MOTOR_NAME),
            ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final SimpleSensor
            REVERSE_LIMIT_SENSOR = SimpleSensor.createDigitalSensor(REVERSE_LIMIT_SENSOR_CHANNEL, REVERSE_LIMIT_SWITCH_NAME),
            FORWARD_LIMIT_SENSOR = SimpleSensor.createDigitalSensor(FORWARD_LIMIT_CHANNEL, FORWARD_LIMIT_SWITCH_NAME),
            DISTANCE_SENSOR = SimpleSensor.createDigitalSensor(DISTANCE_SENSOR_CHANNEL, DISTANCE_SENSOR_NAME);

    private static final double
            INTAKE_MOTOR_GEAR_RATIO = 4,
            ANGLE_MOTOR_GEAR_RATIO = 28;
    static final boolean FOC_ENABLED = true;

    private static final int
            INTAKE_MOTOR_AMOUNT = 1,
            ANGLE_MOTOR_AMOUNT = 1;
    private static final DCMotor
            INTAKE_GEARBOX = DCMotor.getFalcon500(INTAKE_MOTOR_AMOUNT),
            ANGLE_GEARBOX = DCMotor.getKrakenX60(ANGLE_MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double
            INTAKE_LENGTH_METERS = 0.365,
            INTAKE_MASS_KILOGRAMS = 3.26;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(8.34),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(70);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SimpleMotorSimulation INTAKE_SIMULATION = new SimpleMotorSimulation(
            INTAKE_GEARBOX,
            INTAKE_MOTOR_GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final SingleJointedArmSimulation ANGLE_MOTOR_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            INTAKE_LENGTH_METERS,
            INTAKE_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );
    private static final DoubleSupplier
            REVERSE_LIMIT_SENSOR_SIMULATION_SUPPLIER = () -> 0,
            FORWARD_LIMIT_SENSOR_SIMULATION_SUPPLIER = () -> 0,
            DISTANCE_SENSOR_SIMULATION_SUPPLIER = () -> SimulationFieldHandler.isHoldingCoral() ? 1 : 0;

    static final SysIdRoutine.Config ANGLE_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.2).per(Units.Second),
            Units.Volts.of(0.6),
            null
    );

    static final Pose3d INTAKE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0.3234, 0, 0.2944),
            new Rotation3d(0, -2.28, 0)
    );

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d INTAKE_MECHANISM = new SpeedMechanism2d(
            "IntakeMechanism",
            MAXIMUM_DISPLAYABLE_VELOCITY
    );
    static final SingleJointedArmMechanism2d ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            "IntakeAngleMechanism",
            INTAKE_LENGTH_METERS,
            Color.kRed
    );

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.5);
    private static final double
            COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS = 0.2,
            REVERSE_LIMIT_SENSOR_DEBOUNCE_TIME_SECONDS = 0.1,
            FORWARD_LIMIT_SENSOR_DEBOUNCE_TIME_SECONDS = 0.1;
    static final BooleanEvent COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            DISTANCE_SENSOR::getBinaryValue
    ).debounce(COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS);
    private static final BooleanEvent
            REVERSE_LIMIT_SENSOR_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            REVERSE_LIMIT_SENSOR::getBinaryValue
    ).debounce(REVERSE_LIMIT_SENSOR_DEBOUNCE_TIME_SECONDS),
            FORWARD_LIMIT_SENSOR_BOOLEAN_EVENT = new BooleanEvent(
                    CommandScheduler.getInstance().getActiveButtonLoop(),
                    FORWARD_LIMIT_SENSOR::getBinaryValue
            ).debounce(FORWARD_LIMIT_SENSOR_DEBOUNCE_TIME_SECONDS);
    public static Pose3d CORAL_COLLECTION_POSE = new Pose3d(
            new Translation3d(-23.58, 23.73, -20.22),
            new Rotation3d()
    );

    static {
        configureIntakeMotor();
        configureAngleMotor();
        configureLimitSensor(REVERSE_LIMIT_SENSOR, REVERSE_LIMIT_SENSOR_SIMULATION_SUPPLIER, REVERSE_LIMIT_SENSOR_BOOLEAN_EVENT, MINIMUM_ANGLE);
        configureLimitSensor(FORWARD_LIMIT_SENSOR, FORWARD_LIMIT_SENSOR_SIMULATION_SUPPLIER, FORWARD_LIMIT_SENSOR_BOOLEAN_EVENT, MAXIMUM_ANGLE);
        configureDistanceSensor();
    }

    private static void configureIntakeMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.RotorToSensorRatio = INTAKE_MOTOR_GEAR_RATIO;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        INTAKE_MOTOR.applyConfiguration(config);
        INTAKE_MOTOR.setPhysicsSimulation(INTAKE_SIMULATION);

        INTAKE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        INTAKE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        INTAKE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 500 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.0054454 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 3.3247 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.047542 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.35427 : 0.32151;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 12 / config.Slot0.kV : 2;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 8 : 8;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MINIMUM_ANGLE.getRotations();

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXIMUM_ANGLE.getRotations();

        config.CurrentLimits.SupplyCurrentLimit = 60;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ANGLE_MOTOR_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureLimitSensor(SimpleSensor limitSensor, DoubleSupplier simulationSupplier, BooleanEvent booleanEvent, Rotation2d resetPosition) {
        limitSensor.setSimulationSupplier(simulationSupplier);

        booleanEvent.ifHigh(() -> ANGLE_MOTOR.setPosition(resetPosition.getRotations()));
    }

    private static void configureDistanceSensor() {
        DISTANCE_SENSOR.setSimulationSupplier(DISTANCE_SENSOR_SIMULATION_SUPPLIER);
    }

    public enum IntakeState {
        REST(0, MAXIMUM_ANGLE),
        COLLECT(5, MINIMUM_ANGLE),
        EJECT(-5, MINIMUM_ANGLE);

        public final double targetVoltage;
        public final Rotation2d targetAngle;

        IntakeState(double targetVoltage, Rotation2d targetAngle) {
            this.targetVoltage = targetVoltage;
            this.targetAngle = targetAngle;
        }
    }
}