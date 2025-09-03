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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import trigon.hardware.RobotHardwareStats;
import trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import trigon.hardware.simulation.SimpleMotorSimulation;
import trigon.hardware.simulation.SingleJointedArmSimulation;
import trigon.utilities.Conversions;
import trigon.utilities.mechanisms.SingleJointedArmMechanism2d;
import trigon.utilities.mechanisms.SpeedMechanism2d;

public class ArmConstants {
    private static final int
            MASTER_MOTOR_ID = 13,
            FOLLOWER_MOTOR_ID = 14,
            END_EFFECTOR_MOTOR_ID = 15,
            ENCODER_ID = 13;
    private static final String
            MASTER_MOTOR_NAME = "ArmMasterMotor",
            FOLLOWER_MOTOR_NAME = "ArmFollowerMotor",
            END_EFFECTOR_MOTOR_NAME = "EndEffectorMotor",
            ENCODER_NAME = "ArmEncoder";
    static final TalonFXMotor
            ARM_MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            ARM_FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME),
            END_EFFECTOR_MOTOR = new TalonFXMotor(END_EFFECTOR_MOTOR_ID, END_EFFECTOR_MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);

    private static final double
            ARM_GEAR_RATIO = 50,
            END_EFFECTOR_GEAR_RATIO = 17;
    private static final double ANGLE_ENCODER_GRAVITY_OFFSET = 0;
    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET = RobotHardwareStats.isSimulation() ? 0 + Conversions.degreesToRotations(-90) : 0 + Conversions.degreesToRotations(0) - ANGLE_ENCODER_GRAVITY_OFFSET;
    private static final boolean SHOULD_ARM_FOLLOWER_OPPOSE_MASTER = false;
    static final double
            ARM_DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 0.6 : 0,
            ARM_DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 1.5 : 0,
            ARM_DEFAULT_MAXIMUM_JERK = ARM_DEFAULT_MAXIMUM_ACCELERATION * 10;
    static final boolean FOC_ENABLED = true;

    private static final int
            ARM_MOTOR_AMOUNT = 2,
            END_EFFECTOR_MOTOR_AMOUNT = 1;
    private static final DCMotor
            ARM_GEARBOX = DCMotor.getKrakenX60Foc(ARM_MOTOR_AMOUNT),
            END_EFFECTOR_GEARBOX = DCMotor.getKrakenX60Foc(END_EFFECTOR_MOTOR_AMOUNT);
    private static final double
            ARM_LENGTH_METERS = 0.67,
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

    static final SysIdRoutine.Config ARM_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Seconds),
            Units.Volts.of(1.5),
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
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0);

    static {
        configureArmMasterMotor();
        configureArmFollowerMotor();
        configureEndEffectorMotor();
        configureEncoder();
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

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 50 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.0067258 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 6.2 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.063357 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.15048 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = ARM_DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ARM_DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

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

        END_EFFECTOR_MOTOR.applyConfiguration(config);
        END_EFFECTOR_MOTOR.setPhysicsSimulation(END_EFFECTOR_SIMULATION);

        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        END_EFFECTOR_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(ARM_MASTER_MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    public enum ArmState {
        REST(Rotation2d.fromDegrees(0), 0),
        SCORE_L1(Rotation2d.fromDegrees(75), 4),
        SCORE_L2(Rotation2d.fromDegrees(90), 4),
        SCORE_L3(Rotation2d.fromDegrees(90), 4),
        SCORE_L4(Rotation2d.fromDegrees(90), 4),
        SCORE_NET(Rotation2d.fromDegrees(160), 4),
        SCORE_PROCESSOR(Rotation2d.fromDegrees(90), 4),
        SCORE_L1_REVERSE(Rotation2d.fromDegrees(360 - SCORE_L1.targetAngle.getDegrees()), 4),
        SCORE_L2_REVERSE(Rotation2d.fromDegrees(360 - SCORE_L2.targetAngle.getDegrees()), 4),
        SCORE_L3_REVERSE(Rotation2d.fromDegrees(360 - SCORE_L3.targetAngle.getDegrees()), 4),
        SCORE_L4_REVERSE(Rotation2d.fromDegrees(360 - SCORE_L4.targetAngle.getDegrees()), 4),
        SCORE_NET_REVERSE(Rotation2d.fromDegrees(360 - SCORE_NET.targetAngle.getDegrees()), 4),
        SCORE_PROCESSOR_REVERSE(Rotation2d.fromDegrees(360 - SCORE_PROCESSOR.targetAngle.getDegrees()), 4);

        public final Rotation2d targetAngle;
        public final double targetVoltage;

        ArmState(Rotation2d targetAngle, double targetVoltage) {
            this.targetAngle = targetAngle;
            this.targetVoltage = targetVoltage;
        }
    }
}