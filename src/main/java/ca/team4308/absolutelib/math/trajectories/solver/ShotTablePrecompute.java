package ca.team4308.absolutelib.math.trajectories.solver;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.shooter.ShooterConfig;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable;

/**
 * Utility for precomputing trajectory solutions into a lookup table. Generates
 * JSON output for offline use and rehydration into {@link ShotLookupTable}.
 */
public final class ShotTablePrecompute {

    private ShotTablePrecompute() {
    }

    /**
     * Interface for supplying Java-configured precompute settings.
     */
    public interface PrecomputeProfile {

        PrecomputeSpec buildSpec();
    }

    /**
     * Java-configured precompute settings.
     */
    public static final class PrecomputeSpec {

        public FieldBounds bounds;
        public RobotOutline outline;
        public double gridStepMeters = 0.25;

        public double shooterZMeters = 0.5;
        public double targetX = 4.5;
        public double targetY = 4.035;
        public double targetZ = 2.6;
        public double targetRadiusMeters = 0.45;

        public double robotVx = 0.0;
        public double robotVy = 0.0;
        public boolean includeAirResistance = true;
        public ShotInput.ShotPreference shotPreference = ShotInput.ShotPreference.AUTO;
        public int maxCandidates = 50;
        public double minPitchDegrees = 5.0;
        public double maxPitchDegrees = 85.0;
        public double minVelocityMps = 5.0;
        public double maxVelocityMps = 50.0;
        public double angleStepDegrees = 1.0;
        public double minArcHeightMeters = 0.0;
        public double preferredArcHeightMeters = 0.0;
        public double arcBiasStrength = 0.5;
        public boolean collisionCheckEnabled = false;

        public List<PrecomputeConfig.TuningPoint> tuningPoints = new ArrayList<>();

        public TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.defaults();
        public TrajectorySolver.SolveMode solveMode = TrajectorySolver.SolveMode.SWEEP;
        public FlywheelConfig flywheelConfig = null;
        public GamePiece gamePiece = GamePieces.getCurrent();
        public boolean debugEnabled = false;

        public ShooterConfig shooterConfig = null;
        public Runnable solverConstantsApplier = null;

        public String outputPath = "shot-table.json";

        ShotInput toTemplateInput() {
            ShotInput.Builder builder = ShotInput.builder()
                    .shooterPositionMeters(0.0, 0.0, shooterZMeters)
                    .shooterYawDegrees(0.0)
                    .targetPositionMeters(targetX, targetY, targetZ)
                    .targetRadiusMeters(targetRadiusMeters)
                    .robotVelocity(robotVx, robotVy)
                    .includeAirResistance(includeAirResistance)
                    .shotPreference(shotPreference)
                    .maxCandidates(maxCandidates)
                    .pitchRangeDegrees(minPitchDegrees, maxPitchDegrees)
                    .velocityRangeMps(minVelocityMps, maxVelocityMps)
                    .angleStepDegrees(angleStepDegrees)
                    .minArcHeightMeters(minArcHeightMeters)
                    .preferredArcHeightMeters(preferredArcHeightMeters)
                    .arcBiasStrength(arcBiasStrength);

            if (collisionCheckEnabled) {
                builder.collisionCheckEnabled(true);
            }

            return builder.build();
        }

        TrajectorySolver buildSolver() {
            if (solverConstantsApplier != null) {
                solverConstantsApplier.run();
            }
            TrajectorySolver solver = new TrajectorySolver(gamePiece, solverConfig);
            if (tuningPoints != null) {
                for (PrecomputeConfig.TuningPoint tp : tuningPoints) {
                    solver.addTuningPoint(tp.distanceMeters, tp.pitchDegrees, tp.rpm);
                }
            }
            solver.setSolveMode(solveMode);
            solver.setDebugEnabled(debugEnabled);
            if (flywheelConfig != null) {
                solver.setFlywheel(flywheelConfig);
            }
            return solver;
        }
    }

    /**
     * Configuration for precomputing a shot table. Populate this from JSON and
     * pass into {@link #generateFromConfig}.
     */
    public static final class PrecomputeConfig {

        public FieldBounds bounds;
        public RobotOutline outline;
        public double gridStepMeters = 0.25;

        public double shooterZMeters = 0.5;
        public double targetX = 12.405;
        public double targetY = 4.105;  // Center of field
        public double targetZ = 2.0;    // Target height
        public double targetRadiusMeters = 0.45;

        public double robotVx = 0.0;
        public double robotVy = 0.0;
        public boolean includeAirResistance = true;
        public String shotPreference = "AUTO";
        public int maxCandidates = 50;
        public double minPitchDegrees = 5.0;
        public double maxPitchDegrees = 85.0;
        public double minVelocityMps = 5.0;
        public double maxVelocityMps = 50.0;
        public double angleStepDegrees = 1.0;
        public double minArcHeightMeters = 0.0;
        public double preferredArcHeightMeters = 0.0;
        public double arcBiasStrength = 0.5;
        public boolean collisionCheckEnabled = false;

        public List<TuningPoint> tuningPoints = new ArrayList<>();

        public static class TuningPoint {
            public double distanceMeters;
            public double pitchDegrees = -1;
            public double rpm = -1;
        }

        // ── Robot-specific flywheel/motor config ──
        /**
         * Motor name, looked up via
         * {@link ca.team4308.absolutelib.math.trajectories.motor.FRCMotors#getByName}.
         */
        public String motorName = "Kraken X60";
        /**
         * Flywheel wheel diameter in inches.
         */
        public double wheelDiameterInches = 4.0;
        /**
         * Flywheel wheel width in inches.
         */
        public double wheelWidthInches = 2.0;
        /**
         * Motor-to-wheel gear ratio (> 1 = speed up).
         */
        public double gearRatio = 1.0;
        /**
         * Number of motors per wheel.
         */
        public int motorsPerWheel = 1;
        /**
         * Number of flywheel wheels.
         */
        public int wheelCount = 2;
        /**
         * Wheel compression ratio against the ball.
         */
        public double compressionRatio = 0.10;
        /**
         * Wheel arrangement: SINGLE, DUAL_PARALLEL, DUAL_OVER_UNDER.
         */
        public String wheelArrangement = "DUAL_OVER_UNDER";

        // ── Game piece & solver ──
        /**
         * FRC game year for game piece selection (e.g. 2026).
         */
        public int gamePieceYear = 2026;
        /**
         * Solve mode: SWEEP, CONSTRAINT, BISECTION.
         */
        public String solveMode = "SWEEP";
        /**
         * Sweep step in degrees (only for SWEEP mode).
         */
        public double sweepStepDegrees = 1.0;

        // ── Precision mode ──
        /**
         * When true, overrides gridStep/angleStep/maxCandidates for
         * high-fidelity offline precompute.
         */
        public boolean precisionMode = false;

        // ── Alliance / field ──
        /**
         * Full field length in meters (used for alliance mirroring).
         */
        public double fieldLengthMeters = 16.54;
        /**
         * When true, only precomputes one half of the field (minX to
         * fieldCenter).
         */
        public boolean halfFieldOnly = false;

        public String outputPath = "shot-table.json";

        /**
         * Builds a {@link FlywheelConfig} from the flat JSON fields.
         */
        public FlywheelConfig buildFlywheelConfig() {
            ca.team4308.absolutelib.math.trajectories.motor.MotorSpec motor
                    = ca.team4308.absolutelib.math.trajectories.motor.FRCMotors.getByName(motorName);
            if (motor == null) {
                motor = ca.team4308.absolutelib.math.trajectories.motor.FRCMotors.getDefaultShooterMotor();
            }
            FlywheelConfig.WheelArrangement arrangement;
            try {
                arrangement = FlywheelConfig.WheelArrangement.valueOf(
                        wheelArrangement.trim().toUpperCase(Locale.US));
            } catch (IllegalArgumentException e) {
                arrangement = FlywheelConfig.WheelArrangement.DUAL_OVER_UNDER;
            }
            return FlywheelConfig.builder()
                    .name("Precompute Flywheel")
                    .arrangement(arrangement)
                    .wheelDiameterInches(wheelDiameterInches)
                    .wheelWidthInches(wheelWidthInches)
                    .gearRatio(gearRatio)
                    .motor(motor)
                    .motorsPerWheel(motorsPerWheel)
                    .wheelCount(wheelCount)
                    .compressionRatio(compressionRatio)
                    .build();
        }

        /**
         * Builds a {@link TrajectorySolver.SolverConfig} from the JSON fields.
         */
        public TrajectorySolver.SolverConfig buildSolverConfig() {
            TrajectorySolver.SolverConfig.Builder b = TrajectorySolver.SolverConfig.builder();
            b.sweepStepDegrees(sweepStepDegrees);
            if (precisionMode) {
                // High-fidelity offline settings
                b.sweepStepDegrees(0.5);
                b.velocityRefineIterations(12);
            }
            return b.build();
        }

        /**
         * Selects the {@link GamePiece} by year.
         */
        public GamePiece resolveGamePiece() {
            GamePiece piece = GamePieces.getByYear(gamePieceYear);
            return piece != null ? piece : GamePieces.getCurrent();
        }

        /**
         * Parses the solve mode string.
         */
        public TrajectorySolver.SolveMode resolveSolveMode() {
            try {
                return TrajectorySolver.SolveMode.valueOf(solveMode.trim().toUpperCase(Locale.US));
            } catch (IllegalArgumentException e) {
                return TrajectorySolver.SolveMode.SWEEP;
            }
        }

        ShotInput toTemplateInput() {
            int effectiveCandidates = precisionMode ? Math.max(maxCandidates, 100) : maxCandidates;
            double effectiveAngleStep = precisionMode ? Math.min(angleStepDegrees, 0.5) : angleStepDegrees;

            ShotInput.Builder builder = ShotInput.builder()
                    .shooterPositionMeters(0.0, 0.0, shooterZMeters)
                    .shooterYawDegrees(0.0)
                    .targetPositionMeters(targetX, targetY, targetZ)
                    .targetRadiusMeters(targetRadiusMeters)
                    .robotVelocity(robotVx, robotVy)
                    .includeAirResistance(includeAirResistance)
                    .shotPreference(parseShotPreference())
                    .maxCandidates(effectiveCandidates)
                    .pitchRangeDegrees(minPitchDegrees, maxPitchDegrees)
                    .velocityRangeMps(minVelocityMps, maxVelocityMps)
                    .angleStepDegrees(effectiveAngleStep)
                    .minArcHeightMeters(minArcHeightMeters)
                    .preferredArcHeightMeters(preferredArcHeightMeters)
                    .arcBiasStrength(arcBiasStrength);

            if (collisionCheckEnabled) {
                builder.collisionCheckEnabled(true);
            }

            return builder.build();
        }

        private ShotInput.ShotPreference parseShotPreference() {
            return fromPreference(shotPreference);
        }

        public static ShotInput.ShotPreference fromPreference(String preference) {
            if (preference == null) {
                return ShotInput.ShotPreference.AUTO;
            }
            try {
                return ShotInput.ShotPreference.valueOf(preference.trim().toUpperCase(Locale.US));
            } catch (IllegalArgumentException ex) {
                return ShotInput.ShotPreference.AUTO;
            }
        }
    }

    /**
     * Defines the robot footprint and shooter offset in robot coordinates.
     */
    public static final class RobotOutline {

        private final double lengthMeters;
        private final double widthMeters;
        private final double shooterOffsetXMeters;
        private final double shooterOffsetYMeters;

        /**
         * @param lengthMeters robot length (front-back) in meters
         * @param widthMeters robot width (left-right) in meters
         * @param shooterOffsetXMeters shooter offset from robot center (forward
         * +)
         * @param shooterOffsetYMeters shooter offset from robot center (left +)
         */
        @JsonCreator
        public RobotOutline(@JsonProperty("lengthMeters") double lengthMeters,
                @JsonProperty("widthMeters") double widthMeters,
                @JsonProperty("shooterOffsetXMeters") double shooterOffsetXMeters,
                @JsonProperty("shooterOffsetYMeters") double shooterOffsetYMeters) {
            if (lengthMeters <= 0 || widthMeters <= 0) {
                throw new IllegalArgumentException("Robot outline dimensions must be > 0");
            }
            this.lengthMeters = lengthMeters;
            this.widthMeters = widthMeters;
            this.shooterOffsetXMeters = shooterOffsetXMeters;
            this.shooterOffsetYMeters = shooterOffsetYMeters;
        }

        public double getLengthMeters() {
            return lengthMeters;
        }

        public double getWidthMeters() {
            return widthMeters;
        }

        public double getShooterOffsetXMeters() {
            return shooterOffsetXMeters;
        }

        public double getShooterOffsetYMeters() {
            return shooterOffsetYMeters;
        }

        double getHalfLength() {
            return lengthMeters * 0.5;
        }

        double getHalfWidth() {
            return widthMeters * 0.5;
        }
    }

    /**
     * Axis-aligned field bounds for sampling robot positions.
     */
    public static final class FieldBounds {

        private final double minX;
        private final double maxX;
        private final double minY;
        private final double maxY;

        @JsonCreator
        public FieldBounds(@JsonProperty("minX") double minX,
                @JsonProperty("maxX") double maxX,
                @JsonProperty("minY") double minY,
                @JsonProperty("maxY") double maxY) {
            if (maxX < minX || maxY < minY) {
                throw new IllegalArgumentException("Invalid bounds: max must be >= min");
            }
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }

        public double getMinX() {
            return minX;
        }

        public double getMaxX() {
            return maxX;
        }

        public double getMinY() {
            return minY;
        }

        public double getMaxY() {
            return maxY;
        }
    }

    /**
     * Progress callback for long-running precompute runs.
     */
    public interface ProgressListener {

        /**
         * Called after each grid sample is solved.
         *
         * @param current 1-based index of the sample just completed
         * @param total total number of samples
         * @param successCount number of successful solves so far
         * @param skippedCount number of failed/skipped solves so far
         * @param robotX robot center X for this sample (meters)
         * @param robotY robot center Y for this sample (meters)
         * @param shooterX shooter X for this sample (meters)
         * @param shooterY shooter Y for this sample (meters)
         * @param result the TrajectoryResult for this sample, or null if the
         * solve failed
         */
        void onProgress(int current, int total, int successCount, int skippedCount,
                double robotX, double robotY, double shooterX, double shooterY,
                TrajectoryResult result);

        /**
         * Backwards-compatible overload for callers that don't need per-sample
         * data.
         */
        default void onProgress(int current, int total, int successCount, int skippedCount) {
            onProgress(current, total, successCount, skippedCount, 0, 0, 0, 0, null);
        }
    }

    /**
     * Loads a precompute configuration from a JSON file.
     */
    public static PrecomputeConfig loadConfig(Path configPath) throws IOException {
        if (configPath == null) {
            throw new IllegalArgumentException("configPath cannot be null");
        }
        ObjectMapper mapper = new ObjectMapper();
        return mapper.readValue(configPath.toFile(), PrecomputeConfig.class);
    }

    /**
     * Generates a shot table based on a Java-configured spec.
     */
    public static ShotTable generateFromSpec(PrecomputeSpec spec, ProgressListener listener) {
        if (spec == null) {
            throw new IllegalArgumentException("spec cannot be null");
        }
        if (spec.bounds == null) {
            throw new IllegalArgumentException("spec.bounds cannot be null");
        }
        if (spec.outline == null) {
            throw new IllegalArgumentException("spec.outline cannot be null");
        }
        ShotInput template = spec.toTemplateInput();
        return generateForArea(spec.buildSolver(), template, spec.bounds, spec.outline, spec.gridStepMeters, listener);
    }

    /**
     * Generates a shot table based on a JSON configuration.
     */
    public static ShotTable generateFromConfig(TrajectorySolver solver, PrecomputeConfig config,
            ProgressListener listener) {
        if (config == null) {
            throw new IllegalArgumentException("config cannot be null");
        }
        if (config.bounds == null) {
            throw new IllegalArgumentException("config.bounds cannot be null");
        }
        if (config.outline == null) {
            throw new IllegalArgumentException("config.outline cannot be null");
        }
        ShotInput template = config.toTemplateInput();
        return generateForArea(solver, template, config.bounds, config.outline, config.gridStepMeters, listener);
    }

    /**
     * Generates a shot table by sampling robot positions inside a bounding box.
     *
     * @param solver trajectory solver to use
     * @param template base shot input (target height, config, obstacles)
     * @param bounds axis-aligned field bounds in meters
     * @param outline robot footprint and shooter offset
     * @param gridStepMeters spacing between sampled robot positions
     * @param listener optional progress listener (nullable)
     * @return generated shot table containing valid entries
     */
    public static ShotTable generateForArea(TrajectorySolver solver, ShotInput template,
            FieldBounds bounds, RobotOutline outline, double gridStepMeters,
            ProgressListener listener) {
        if (solver == null) {
            throw new IllegalArgumentException("solver cannot be null");
        }
        if (template == null) {
            throw new IllegalArgumentException("template cannot be null");
        }
        if (bounds == null) {
            throw new IllegalArgumentException("bounds cannot be null");
        }
        if (outline == null) {
            throw new IllegalArgumentException("outline cannot be null");
        }
        if (gridStepMeters <= 0) {
            throw new IllegalArgumentException("gridStepMeters must be > 0");
        }

        double minX = bounds.getMinX() + outline.getHalfLength();
        double maxX = bounds.getMaxX() - outline.getHalfLength();
        double minY = bounds.getMinY() + outline.getHalfWidth();
        double maxY = bounds.getMaxY() - outline.getHalfWidth();

        if (maxX < minX || maxY < minY) {
            throw new IllegalArgumentException("Bounds too small for robot outline");
        }

        int countX = (int) Math.floor((maxX - minX) / gridStepMeters) + 1;
        int countY = (int) Math.floor((maxY - minY) / gridStepMeters) + 1;
        int total = Math.max(0, countX * countY);

        List<ShotTableEntry> entries = Collections.synchronizedList(new ArrayList<>());
        java.util.concurrent.atomic.AtomicInteger skipped = new java.util.concurrent.atomic.AtomicInteger(0);
        java.util.concurrent.atomic.AtomicInteger success = new java.util.concurrent.atomic.AtomicInteger(0);
        java.util.concurrent.atomic.AtomicInteger index = new java.util.concurrent.atomic.AtomicInteger(0);

        double targetX = template.getTargetX();
        double targetY = template.getTargetY();
        double targetZ = template.getTargetZ();

        java.util.stream.IntStream.range(0, total).parallel().forEach(i -> {
            int ix = i / countY;
            int iy = i % countY;

            double robotX = minX + ix * gridStepMeters;
            double robotY = minY + iy * gridStepMeters;

            double yaw = Math.atan2(targetY - robotY, targetX - robotX);
            double cos = Math.cos(yaw);
            double sin = Math.sin(yaw);
            double shooterX = robotX + outline.getShooterOffsetXMeters() * cos
                    - outline.getShooterOffsetYMeters() * sin;
            double shooterY = robotY + outline.getShooterOffsetXMeters() * sin
                    + outline.getShooterOffsetYMeters() * cos;

            double shooterYaw = Math.atan2(targetY - shooterY, targetX - shooterX);

            ShotInput input = buildInput(template, shooterX, shooterY, template.getShooterZ(), shooterYaw,
                    targetX, targetY, targetZ, template.getTargetRadius());

            TrajectoryResult result = solver.solve(input);
            if (result != null && result.isSuccess()) {
                success.incrementAndGet();
                entries.add(new ShotTableEntry(
                        robotX, robotY,
                        shooterX, shooterY,
                        input.getHorizontalDistanceMeters(),
                        shooterYaw,
                        result.getPitchAngleDegrees(),
                        result.getRecommendedRpm(),
                        result.getConfidenceScore(),
                        result.getRequiredVelocityMps(),
                        result.getTimeOfFlightSeconds()));
            } else {
                skipped.incrementAndGet();
            }

            int currIndex = index.incrementAndGet();
            if (listener != null) {
                listener.onProgress(currIndex, total, success.get(), skipped.get(),
                        robotX, robotY, shooterX, shooterY, result);
            }
        });

            // Optionally prune the table to keep only the highest-quality shots.
            // This helps control output size while ensuring the remaining entries
            // represent the best overall trajectories.
            int maxEntries = 4700;
            if (entries.size() > maxEntries) {
                entries.sort((a, b) -> Double.compare(b.confidenceScore, a.confidenceScore));
                int pruned = entries.size() - maxEntries;
                entries.subList(maxEntries, entries.size()).clear();
                skipped.addAndGet(pruned);
            }

        return new ShotTable(bounds, outline, gridStepMeters, new ArrayList<>(entries), skipped.get());
    }

    /**
     * Writes the shot table to disk as JSON.
     */
    public static void writeJson(Path outputPath, ShotTable table) throws IOException {
        if (outputPath == null) {
            throw new IllegalArgumentException("outputPath cannot be null");
        }
        if (table == null) {
            throw new IllegalArgumentException("table cannot be null");
        }
        Files.writeString(outputPath, table.toJson(), StandardCharsets.UTF_8);
    }

    /**
     * Loads a precomputed shot table from a JSON file.
     *
     * @param inputPath path to the JSON file
     * @return the deserialized ShotTable
     * @throws IOException if reading or parsing fails
     */
    public static ShotTable loadTable(Path inputPath) throws IOException {
        if (inputPath == null) {
            throw new IllegalArgumentException("inputPath cannot be null");
        }

        // This is a basic parser. For a real environment using Jackson:
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(inputPath.toFile());

        JsonNode meta = root.get("metadata");
        double gridStep = meta.get("gridStepMeters").asDouble();
        int skipped = meta.has("skippedCount") ? meta.get("skippedCount").asInt() : 0;

        JsonNode b = meta.get("bounds");
        FieldBounds bounds = new FieldBounds(b.get("minX").asDouble(), b.get("maxX").asDouble(),
                b.get("minY").asDouble(), b.get("maxY").asDouble());

        JsonNode o = meta.get("robotOutline");
        RobotOutline outline = new RobotOutline(o.get("lengthMeters").asDouble(), o.get("widthMeters").asDouble(),
                o.get("shooterOffsetX").asDouble(), o.get("shooterOffsetY").asDouble());

        List<ShotTableEntry> entries = new ArrayList<>();
        JsonNode entriesNode = root.get("entries");
        if (entriesNode != null && entriesNode.isArray()) {
            for (JsonNode e : entriesNode) {
                entries.add(new ShotTableEntry(
                        e.get("robotX").asDouble(), e.get("robotY").asDouble(),
                        e.get("shooterX").asDouble(), e.get("shooterY").asDouble(),
                        e.get("distanceMeters").asDouble(), e.get("yawRadians").asDouble(),
                        e.get("pitchDegrees").asDouble(), e.get("rpm").asDouble(),
                        e.has("confidenceScore") ? e.get("confidenceScore").asDouble() : 0.0,
                        e.get("exitVelocityMps").asDouble(), e.get("timeOfFlightSeconds").asDouble()
                ));
            }
        }

        ShotTable table = new ShotTable(bounds, outline, gridStep, entries, skipped);

        if (meta.has("version")) {
            table.setExtendedMeta(
                    meta.get("version").asText(),
                    meta.has("gameYear") ? meta.get("gameYear").asInt() : 2026,
                    meta.has("motorName") ? meta.get("motorName").asText() : "",
                    meta.has("fieldLengthMeters") ? meta.get("fieldLengthMeters").asDouble() : 16.54,
                    meta.has("halfFieldOnly") ? meta.get("halfFieldOnly").asBoolean() : false
            );
        }
        return table;
    }

    private static ShotInput buildInput(ShotInput template,
            double shooterX, double shooterY, double shooterZ, double shooterYaw,
            double targetX, double targetY, double targetZ, double targetRadius) {
        ShotInput.Builder builder = ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterZ)
                .shooterYawRadians(shooterYaw)
                .targetPositionMeters(targetX, targetY, targetZ)
                .targetRadiusMeters(targetRadius)
                .robotVelocity(template.getEffectiveRobotVx(), template.getEffectiveRobotVy())
                .includeAirResistance(template.isIncludeAirResistance())
                .shotPreference(template.getShotPreference())
                .maxCandidates(template.getMaxCandidates())
                .pitchRangeDegrees(template.getMinPitchDegrees(), template.getMaxPitchDegrees())
                .velocityRangeMps(template.getMinVelocityMps(), template.getMaxVelocityMps())
                .angleStepDegrees(template.getAngleStepDegrees())
                .minArcHeightMeters(template.getMinArcHeightMeters())
                .preferredArcHeightMeters(template.getPreferredArcHeightMeters())
                .arcBiasStrength(template.getArcBiasStrength())
                .obstacles(template.getObstacles());

        if (template.isCollisionCheckEnabled()) {
            builder.collisionCheckEnabled(true);
        }

        return builder.build();
    }

    /**
     * Data container for a precomputed shot table.
     */
    public static final class ShotTable {

        private final FieldBounds bounds;
        private final RobotOutline outline;
        private final double gridStepMeters;
        private final List<ShotTableEntry> entries;
        private final int skippedCount;
        private final Instant generatedAt;

        // Extended metadata
        private String version = "2.2.0";
        private int gameYear = 2026;
        private String motorName = "";
        private double fieldLengthMeters = 16.54;
        private boolean halfFieldOnly = false;

        private ShotTable(FieldBounds bounds, RobotOutline outline, double gridStepMeters,
                List<ShotTableEntry> entries, int skippedCount) {
            this.bounds = bounds;
            this.outline = outline;
            this.gridStepMeters = gridStepMeters;
            this.entries = Collections.unmodifiableList(new ArrayList<>(entries));
            this.skippedCount = skippedCount;
            this.generatedAt = Instant.now();
        }

        /**
         * Sets extended metadata (called by generateForArea).
         */
        void setExtendedMeta(String version, int gameYear, String motorName,
                double fieldLengthMeters, boolean halfFieldOnly) {
            this.version = version;
            this.gameYear = gameYear;
            this.motorName = motorName;
            this.fieldLengthMeters = fieldLengthMeters;
            this.halfFieldOnly = halfFieldOnly;
        }

        public FieldBounds getBounds() {
            return bounds;
        }

        public RobotOutline getOutline() {
            return outline;
        }

        public double getGridStepMeters() {
            return gridStepMeters;
        }

        public List<ShotTableEntry> getEntries() {
            return entries;
        }

        public int getSkippedCount() {
            return skippedCount;
        }

        public Instant getGeneratedAt() {
            return generatedAt;
        }

        public double getFieldLengthMeters() {
            return fieldLengthMeters;
        }

        public boolean isHalfFieldOnly() {
            return halfFieldOnly;
        }

        // ── Coordinate-based lookup ──
        /**
         * Finds the closest entry to the given robot position.
         *
         * @param robotX robot X coordinate in meters
         * @param robotY robot Y coordinate in meters
         * @return the nearest ShotTableEntry, or null if empty
         */
        public ShotTableEntry lookupNearest(double robotX, double robotY) {
            if (entries.isEmpty()) {
                return null;
            }
            ShotTableEntry best = null;
            double bestDist = Double.MAX_VALUE;
            for (ShotTableEntry e : entries) {
                double dx = e.robotX - robotX;
                double dy = e.robotY - robotY;
                double d2 = dx * dx + dy * dy;
                if (d2 < bestDist) {
                    bestDist = d2;
                    best = e;
                }
            }
            return best;
        }

        /**
         * Mirrors the robot X coordinate for the opposite alliance and looks up
         * the nearest precomputed entry. Use this when the table was generated
         * for one alliance and you need to look up shots from the other.
         *
         * <p>
         * The mirroring formula is:
         * {@code mirroredX = fieldLengthMeters - robotX}.
         *
         * @param robotX robot X coordinate in meters (field-relative)
         * @param robotY robot Y coordinate in meters (field-relative)
         * @return the nearest ShotTableEntry after mirroring, or null if empty
         */
        public ShotTableEntry lookupMirrored(double robotX, double robotY) {
            double mirroredX = fieldLengthMeters - robotX;
            return lookupNearest(mirroredX, robotY);
        }

        /**
         * Builds a {@link ShotLookupTable} using the precomputed entries.
         */
        public ShotLookupTable toLookupTable() {
            ShotLookupTable table = new ShotLookupTable();
            for (ShotTableEntry entry : entries) {
                table.addEntry(entry.distanceMeters, entry.pitchDegrees, entry.rpm, entry.timeOfFlightSeconds);
            }
            return table;
        }

        /**
         * Serializes the table as JSON for offline storage.
         */
        public String toJson() {
            StringBuilder sb = new StringBuilder();
            sb.append("{\n");
            sb.append("  \"metadata\": {\n");
            sb.append(String.format(Locale.US, "    \"version\": \"%s\",\n", version));
            sb.append(String.format(Locale.US, "    \"gameYear\": %d,\n", gameYear));
            sb.append(String.format(Locale.US, "    \"motorName\": \"%s\",\n", motorName != null ? motorName : ""));
            sb.append(String.format(Locale.US, "    \"fieldLengthMeters\": %.3f,\n", fieldLengthMeters));
            sb.append(String.format(Locale.US, "    \"halfFieldOnly\": %b,\n", halfFieldOnly));
            sb.append(String.format(Locale.US, "    \"gridStepMeters\": %.4f,\n", gridStepMeters));
            sb.append(String.format(Locale.US, "    \"entryCount\": %d,\n", entries.size()));
            sb.append(String.format(Locale.US, "    \"skippedCount\": %d,\n", skippedCount));
            sb.append(String.format(Locale.US, "    \"bounds\": {\"minX\": %.3f, \"maxX\": %.3f, \"minY\": %.3f, \"maxY\": %.3f},\n",
                    bounds.getMinX(), bounds.getMaxX(), bounds.getMinY(), bounds.getMaxY()));
            sb.append(String.format(Locale.US, "    \"robotOutline\": {\"lengthMeters\": %.3f, \"widthMeters\": %.3f, \"shooterOffsetX\": %.3f, \"shooterOffsetY\": %.3f},\n",
                    outline.getLengthMeters(), outline.getWidthMeters(),
                    outline.getShooterOffsetXMeters(), outline.getShooterOffsetYMeters()));
            sb.append(String.format("    \"generatedAt\": \"%s\"\n", generatedAt));
            sb.append("  },\n");
            sb.append("  \"entries\": [\n");

            for (int i = 0; i < entries.size(); i++) {
                ShotTableEntry entry = entries.get(i);
                sb.append("    {");
                sb.append(String.format(Locale.US, "\"robotX\": %.3f, ", entry.robotX));
                sb.append(String.format(Locale.US, "\"robotY\": %.3f, ", entry.robotY));
                sb.append(String.format(Locale.US, "\"shooterX\": %.3f, ", entry.shooterX));
                sb.append(String.format(Locale.US, "\"shooterY\": %.3f, ", entry.shooterY));
                sb.append(String.format(Locale.US, "\"distanceMeters\": %.4f, ", entry.distanceMeters));
                sb.append(String.format(Locale.US, "\"yawRadians\": %.5f, ", entry.yawRadians));
                sb.append(String.format(Locale.US, "\"pitchDegrees\": %.3f, ", entry.pitchDegrees));
                sb.append(String.format(Locale.US, "\"rpm\": %.1f, ", entry.rpm));
                sb.append(String.format(Locale.US, "\"confidenceScore\": %.1f, ", entry.confidenceScore));
                sb.append(String.format(Locale.US, "\"exitVelocityMps\": %.3f, ", entry.exitVelocityMps));
                sb.append(String.format(Locale.US, "\"timeOfFlightSeconds\": %.3f", entry.timeOfFlightSeconds));
                sb.append("}");
                if (i < entries.size() - 1) {
                    sb.append(",");
                }
                sb.append("\n");
            }

            sb.append("  ]\n");
            sb.append("}\n");
            return sb.toString();
        }
    }

    /**
     * Single precomputed entry for a robot pose.
     */
    public static final class ShotTableEntry {

        public final double robotX;
        public final double robotY;
        public final double shooterX;
        public final double shooterY;
        public final double distanceMeters;
        public final double yawRadians;
        public final double pitchDegrees;
        public final double rpm;
        public final double confidenceScore;
        public final double exitVelocityMps;
        public final double timeOfFlightSeconds;

        public ShotTableEntry(double robotX, double robotY,
                double shooterX, double shooterY,
                double distanceMeters, double yawRadians,
                double pitchDegrees, double rpm,
                double confidenceScore,
                double exitVelocityMps, double timeOfFlightSeconds) {
            this.robotX = robotX;
            this.robotY = robotY;
            this.shooterX = shooterX;
            this.shooterY = shooterY;
            this.distanceMeters = distanceMeters;
            this.yawRadians = yawRadians;
            this.pitchDegrees = pitchDegrees;
            this.rpm = rpm;
            this.confidenceScore = confidenceScore;
            this.exitVelocityMps = exitVelocityMps;
            this.timeOfFlightSeconds = timeOfFlightSeconds;
        }
    }
}
