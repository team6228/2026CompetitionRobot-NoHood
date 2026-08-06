package frc.robot.subsystems.Swerve;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveContants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.ShooterTestSubsystem;

public class DriveTrain extends SubsystemBase {

    private final Timer timer = new Timer();
    // ── Saha sabiti ───────────────────────────────────────────────────────────
    private static final double FIELD_LENGTH = 16.54;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveModule frontLeft  = new SwerveModule(DriveContants.flDriveCanID, DriveContants.flAngleCanID, DriveContants.flChassisAngularOffset, "fl", 0.01958944275,  3.3008210659);
    private final SwerveModule frontRight = new SwerveModule(DriveContants.frDriveCanID, DriveContants.frAngleCanID, DriveContants.frChassisAngularOffset, "fr", 0.01469208206,  3.29592370986);
    private final SwerveModule backLeft   = new SwerveModule(DriveContants.blDriveCanID, DriveContants.blAngleCanID, DriveContants.blChassisAngularOffset, "bl", 0.01958944275,  3.3008210659);
    private final SwerveModule backRight  = new SwerveModule(DriveContants.brDriveCanID, DriveContants.brAngleCanID, DriveContants.brChassisAngularOffset, "br", 0.01469208206,  3.29102635383);

    // ── Control / Estimation ──────────────────────────────────────────────────
    private final PIDController turnPID = new PIDController(0.01, 0, 0);
    private final SwerveDrivePoseEstimator poseEstimator;

    private QuestNavSubsystem questNav;

    // ── Telemetry ─────────────────────────────────────────────────────────────
    private final Field2d m_field = new Field2d();

    private final StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("RobotPoseStruct", Pose2d.struct)
            .publish();

    private final StructArrayPublisher<SwerveModuleState> statePublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
            .publish();

    // ── Auto pose chooser — tüm pose'lar BLUE-ORIGIN'de ─────────────────────
    private final SendableChooser<Pose2d> autoPosChooser = new SendableChooser<>();

    // ── Snapshot — her döngüde bir kez alınır, tüm metotlar bunu kullanır ───
    // Bu sayede driveAtTarget() ve getDynamicShotParameters() aynı "an"ı görür.
    private Pose2d       snapshotPose   = new Pose2d();
    private ChassisSpeeds snapshotFieldSpeeds = new ChassisSpeeds();

    private final Timer oscillateTimer        = new Timer();
    private boolean     oscillateTimerStarted = false;

    /** Her yönde kaç saniye gidileceği. Toplam periyot = 2 * kOscillatePeriod */
    private static final double kOscillatePeriod = 0.3;
 
    /** Sallanma hızı — maxSpeed'in bu oranı kadar (0..1). */
    private static final double kOscillateSpeed  = 0.7;

    // ─────────────────────────────────────────────────────────────────────────
    public DriveTrain() {
        autoPosChooser.setDefaultOption("Center", new Pose2d(3.5, 4.0, Rotation2d.fromDegrees(0)));
        autoPosChooser.addOption(       "Left",   new Pose2d(3.5, 5.1, Rotation2d.fromDegrees(0)));
        autoPosChooser.addOption(       "Right",  new Pose2d(3.5, 2.9, Rotation2d.fromDegrees(0)));

        SmartDashboard.putData("Auto Start Pose", autoPosChooser);
        SmartDashboard.putData("Update Pose",     new InstantCommand(this::resetPoseToSelected).ignoringDisable(true));
        SmartDashboard.putData("Field",           m_field);

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveContants.kDriveKinematics,
            getRotation2d(),
            getModulePositions(),
            autoPosChooser.getSelected()
        );
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999));

        turnPID.enableContinuousInput(-180, 180);

        configureAutoBuilder();
    }

    // ── PathPlanner ───────────────────────────────────────────────────────────

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(5.65, 0.0, 0.0),
                    new PIDConstants(5.65, 0.0, 0.0)
                ),
                config,
                this::isRedAlliance,
                this
            );

        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config failed: " + e.getMessage(), true);
        }
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red)
            .orElse(false);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Pose estimator güncelle
        poseEstimator.update(getRotation2d(), getModulePositions());

        // ── SNAPSHOT: Bu döngüdeki pose ve hız bir kez alınır.
        //    driveAtTarget() ve ShooterSubsystem.getDynamicShotParameters()
        //    aynı execute() döngüsünde bu snapshot'ı kullanır → tutarlı "an".
        snapshotPose        = poseEstimator.getEstimatedPosition();
        snapshotFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            getRobotRelativeSpeeds(),
            snapshotPose.getRotation()
        );

        posePublisher.set(snapshotPose);
        m_field.setRobotPose(snapshotPose);

        frontLeft.updateCalibration();
        frontRight.updateCalibration();
        backLeft.updateCalibration();
        backRight.updateCalibration();

        statePublisher.set(new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        });

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        String allianceStr = alliance.isPresent() ? alliance.get().toString() : "UNKNOWN";

        SmartDashboard.putString("Alliance",       allianceStr);
        SmartDashboard.putNumber("Robot Heading",  getHeading());
    }

    // ── Drive methods ─────────────────────────────────────────────────────────

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed   * DriveContants.maxSpeedMetersPerSecond,
                ySpeed   * DriveContants.maxSpeedMetersPerSecond,
                rotation * DriveContants.maxAngularSpeed,
                snapshotPose.getRotation())
            : new ChassisSpeeds(
                xSpeed   * DriveContants.maxSpeedMetersPerSecond,
                ySpeed   * DriveContants.maxSpeedMetersPerSecond,
                rotation * DriveContants.maxAngularSpeed);

        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public Pose2d getResetPose() {
        return autoPosChooser.getSelected();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Shoot-on-the-move sürüş modu.
     *
     * FIX 1 — Koordinat çerçevesi tutarlılığı:
     *   Önceki versiyonda localX/localY manuel matris çarpımıyla hesaplanıyordu,
     *   fieldSpeeds ise ayrı bir fromRobotRelativeSpeeds() çağrısıyla alınıyordu.
     *   İkisi farklı "an"dan okuyabiliyordu. Artık her ikisi de periodic()'te
     *   alınan snapshotPose ve snapshotFieldSpeeds üzerinden çalışıyor.
     *
     * FIX 2 — Field-frame aim offset:
     *   Önceki versiyonda angleOffset, robot-frame lateral hızından (realLocalVy)
     *   türetilip field-frame açısına (atan2) ekleniyor; bu iki farklı referans
     *   çerçevesini karıştırıyordu.
     *   Düzeltme: hem hedef vektörü hem de lateral hız hesabı tamamen field-frame'de
     *   yapılıyor. Hub yönüne dik bileşen (lateralVelocity) artık field-frame cross
     *   product ile elde ediliyor — robot yönünden bağımsız, her zaman doğru işaret.
     *
     * @param xSpeed  Joystick X (field-relative, -1..1)
     * @param ySpeed  Joystick Y (field-relative, -1..1)
     */
    
    public void setX() {
        frontLeft.setDesiredState( new SwerveModuleState(0, Rotation2d.fromDegrees( 45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(  new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState( new SwerveModuleState(0, Rotation2d.fromDegrees( 45)));
    }

    // ── Module helpers ────────────────────────────────────────────────────────

    public void setModuleStates(SwerveModuleState[] desiredStates) { desaturateAndSet(desiredStates); }

    private void desaturateAndSet(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveContants.maxSpeedMetersPerSecond);
        frontLeft.setDesiredState( states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(  states[2]);
        backRight.setDesiredState( states[3]);
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    // ── Pose / Odometry ───────────────────────────────────────────────────────

    public Pose2d getPose() {
        return snapshotPose;
    }

    /**
     * Snapshot'tan field-relative hız döndürür.
     * ShooterSubsystem bu metodu getDynamicShotParameters() içinde kullanır —
     * driveAtTarget() ile aynı "an"ı paylaşırlar.
     */
    public ChassisSpeeds getSnapshotFieldSpeeds() {
        return snapshotFieldSpeeds;
    }

    /**
     * PathPlanner tarafından çağrılır.
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
            getRotation2d(),
            getModulePositions(),
            pose
        );
    }

    public void resetPoseToSelected() {
        Pose2d pose = autoPosChooser.getSelected();
        if (pose == null) return;

        poseEstimator.resetPosition(
            getRotation2d(),
            getModulePositions(),
            pose
        );

        if (questNav != null) questNav.zeroPose(pose);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    // ── Gyro ──────────────────────────────────────────────────────────────────

    public Rotation2d getRotation2d() {
        double angle = navx.getAngle();
        return Rotation2d.fromDegrees(DriveContants.gyroReversed ? angle : -angle);
    }

    public double getHeading() {
        return snapshotPose.getRotation().getDegrees();
    }

    public void zeroHeading() {
        navx.reset();
        if (questNav != null) questNav.zeroPose(getPose());
    }

    // ── Misc ──────────────────────────────────────────────────────────────────

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveContants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),  frontRight.getState(),
            backLeft.getState(),   backRight.getState()
        );
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setQuestNav(QuestNavSubsystem questNav) {
        this.questNav = questNav;
    }

    public double getFieldRelativeVX() { return snapshotFieldSpeeds.vxMetersPerSecond; }
    public double getFieldRelativeVY() { return snapshotFieldSpeeds.vyMetersPerSecond; }


    public void driveAtTarget(double xSpeed, double ySpeed) {
        // ── 1. Hedef koordinatları (blue-origin) ─────────────────────────────
        double targetX;
        double targetY;

    
        targetX = 4.625594;
        targetY = 4.034536;
        

        Pose2d currentPose = getPose();

        // ── 2. Hedefe vektör ──────────────────────────────────────────────────
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();
        double distanceToTarget = Math.hypot(dx, dy);

        // ── 3. Robot frame'ine çevir ──────────────────────────────────────────
        Rotation2d robotRot = currentPose.getRotation();
        double localX =  dx * robotRot.getCos() + dy * robotRot.getSin();
        double localY = -dx * robotRot.getSin() + dy * robotRot.getCos();

        // ── 4. Gerçek robot hızı ──────────────────────────────────────────────
        ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, getRotation2d());

        double realVx    = fieldSpeeds.vxMetersPerSecond;
        double realVy    = fieldSpeeds.vyMetersPerSecond;
        double realSpeed = Math.hypot(realVx, realVy);

        double realLocalVy = -realVx * robotRot.getSin() + realVy * robotRot.getCos();

        // ── 5. Aim offset ─────────────────────────────────────────────────────
        boolean isMoving = realSpeed > 0.08;
        double angleOffset = 0.0;

        if (isMoving && localX > 0.0) {
            double velocityOffset   = MathUtil.clamp(realLocalVy * 3.0, -6.0, 6.0);
            double distanceFactor   = MathUtil.clamp(distanceToTarget / 4.0, 0.0, 1.0);
            double positionalOffset = MathUtil.clamp(localY * 1.2 * distanceFactor, -5.0, 5.0);
            double speedFactor      = MathUtil.clamp(realSpeed / DriveContants.maxSpeedMetersPerSecond, 0.0, 1.0);
            angleOffset = (velocityOffset + positionalOffset) * speedFactor;
        }

        // ── 6. PID dönüş ──────────────────────────────────────────────────────
        double angleToTarget  = Math.toDegrees(Math.atan2(dy, dx)) + angleOffset;
        double angleError     = Math.IEEEremainder(getHeading() - angleToTarget, 360);
        double rotationOutput = MathUtil.clamp(turnPID.calculate(angleError, 0), -0.5, 0.5);

        if (Math.abs(angleError) < 0.5) rotationOutput = 0;

        // ── 7. Field-relative sürüş ───────────────────────────────────────────
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * DriveContants.maxSpeedMetersPerSecond,
            ySpeed * DriveContants.maxSpeedMetersPerSecond,
            rotationOutput * DriveContants.maxAngularSpeed,
            getRotation2d()
        );

        var states = DriveContants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveContants.maxSpeedMetersPerSecond);
        setModuleStates(states);
    }

        public void oscillate() {
        // İlk çağrıda timer'ı başlat
        if (!oscillateTimerStarted) {
            oscillateTimer.reset();
            oscillateTimer.start();
            oscillateTimerStarted = true;
        }
 
        double t = oscillateTimer.get();
 
        // Periyot dolunca sıfırla
        if (t >= kOscillatePeriod * 2.0) {
            oscillateTimer.reset();
            t = 0.0;
        }
 
        // İlk yarı ileri, ikinci yarı geri — robot-relative
        double forwardSpeed = (t < kOscillatePeriod) ? kOscillateSpeed : -kOscillateSpeed;
 
        ChassisSpeeds speeds = new ChassisSpeeds(
            forwardSpeed * DriveContants.maxSpeedMetersPerSecond,
            0.0,
            0.0
        );
 
        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public void resetOscillate() {
        oscillateTimer.stop();
        oscillateTimer.reset();
        oscillateTimerStarted = false;
    }


    public Command runAuto(FeederSubsystem feeder, ShooterTestSubsystem shooter) {

    return Commands.sequence(

        // 1️⃣ Timer reset
        new InstantCommand(() -> {
            timer.reset(); 
            timer.start();
        }),

        // 2️⃣ 0–1.5s geri git
        new RunCommand(() -> drive(-0.5, 0, 0, true), this)
            .withTimeout(1),

        // 3️⃣ aim (durarak)
        new RunCommand(() -> driveAtTarget(0, 0), this)
            .withTimeout(1.5),

        // 4️⃣ yavaş geri + aim
        new RunCommand(() -> driveAtTarget(-0.2, 0), this)
            .withTimeout(2.0),

        // 5️⃣ shoot
        new ParallelCommandGroup(
            new RunCommand(() -> { shooter.shoot(); shooter.enablePID(); }, shooter),
            new WaitUntilCommand(() -> shooter.isReady())
                .andThen(new RunCommand(() -> feeder.feedShooter(), feeder))
        ).withTimeout(2.5),

        // 6️⃣ stop
        new InstantCommand(() -> {
            drive(0,0,0,true);
            shooter.stopShooter();
            shooter.disablePID();
            feeder.stopMotors();
        })
    );
}
}