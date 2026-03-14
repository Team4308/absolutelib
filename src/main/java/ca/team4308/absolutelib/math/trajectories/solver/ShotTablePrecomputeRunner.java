package ca.team4308.absolutelib.math.trajectories.solver;

import java.nio.file.Path;

/**
 * Example runner for generating a JSON shot table on a desktop JVM.
 * Provide a JSON config file to control bounds, outline, target, and solver settings.
 */
public final class ShotTablePrecomputeRunner {

    private ShotTablePrecomputeRunner() {
    }

    public static void main(String[] args) throws Exception {
        String configArg = args.length > 0 ? args[0] : "shot-precompute.json";
        ShotTablePrecompute.PrecomputeSpec spec;
        Path outputPath;

        if (configArg.toLowerCase().endsWith(".json")) {
            Path configPath = Path.of(configArg);
            ShotTablePrecompute.PrecomputeConfig config = ShotTablePrecompute.loadConfig(configPath);
            spec = new ShotTablePrecompute.PrecomputeSpec();
            spec.bounds = config.bounds;
            spec.outline = config.outline;
            spec.gridStepMeters = config.gridStepMeters;
            spec.shooterZMeters = config.shooterZMeters;
            spec.targetX = config.targetX;
            spec.targetY = config.targetY;
            spec.targetZ = config.targetZ;
            spec.targetRadiusMeters = config.targetRadiusMeters;
            spec.robotVx = config.robotVx;
            spec.robotVy = config.robotVy;
            spec.includeAirResistance = config.includeAirResistance;
            spec.shotPreference = ShotTablePrecompute.PrecomputeConfig.fromPreference(config.shotPreference);
            spec.maxCandidates = config.maxCandidates;
            spec.minPitchDegrees = config.minPitchDegrees;
            spec.maxPitchDegrees = config.maxPitchDegrees;
            spec.minVelocityMps = config.minVelocityMps;
            spec.maxVelocityMps = config.maxVelocityMps;
            spec.angleStepDegrees = config.angleStepDegrees;
            spec.minArcHeightMeters = config.minArcHeightMeters;
            spec.preferredArcHeightMeters = config.preferredArcHeightMeters;
            spec.arcBiasStrength = config.arcBiasStrength;
            spec.collisionCheckEnabled = config.collisionCheckEnabled;

            outputPath = Path.of(args.length > 1 ? args[1] : config.outputPath);
        } else {
            ShotTablePrecompute.PrecomputeProfile profile =
                    (ShotTablePrecompute.PrecomputeProfile) Class.forName(configArg)
                            .getDeclaredConstructor().newInstance();
            spec = profile.buildSpec();
            outputPath = Path.of(args.length > 1 ? args[1] : spec.outputPath);
        }

    ShotTablePrecompute.ShotTable table = ShotTablePrecompute.generateFromSpec(
        spec,
                (current, total, success, skipped) -> {
                    if (current == 1 || current % 25 == 0 || current == total) {
                        System.out.printf("Progress %d/%d (success=%d, skipped=%d)\n",
                                current, total, success, skipped);
                    }
                });

        ShotTablePrecompute.writeJson(outputPath, table);
        System.out.printf("Wrote %d entries to %s (skipped=%d)\n",
                table.getEntries().size(), outputPath, table.getSkippedCount());
    }
}
