package ca.team4308.absolutelib.math.trajectories.solver;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import javax.swing.*;

import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Example runner for generating a JSON shot table on a desktop JVM. Provide a
 * JSON config file to control bounds, outline, target, and solver settings.
 */
public final class ShotTablePrecomputeRunner {

    // ── Colour palette ──────────────────────────────────────────────────
    private static final Color BG_DARK = new Color(0x1E1E2E);
    private static final Color GRID_COLOR = new Color(0x33, 0x33, 0x55, 100);
    private static final Color GRID_LABEL_COLOR = new Color(0x88, 0x88, 0xAA);
    private static final Color BORDER_COLOR = new Color(0x55, 0x55, 0x88);
    private static final Color ROBOT_COLOR = new Color(0x42, 0x9B, 0xF5);
    private static final Color SHOOTER_COLOR = new Color(0x00, 0xDD, 0xDD);
    private static final Color TARGET_COLOR = new Color(0xFF, 0x66, 0x33);
    private static final Color TRAJ_START = new Color(0x33, 0xFF, 0x66);
    private static final Color TRAJ_END = new Color(0xFF, 0xDD, 0x33);
    private static final Color SUCCESS_DOT = new Color(0x33, 0xFF, 0x66, 60);
    private static final Color SKIP_DOT = new Color(0xFF, 0x44, 0x44, 40);
    private static final Color INFO_BG = new Color(0x26, 0x26, 0x3A);
    private static final Color INFO_TEXT = new Color(0xCC, 0xCC, 0xEE);
    private static final Color INFO_HEADER = new Color(0x88, 0xBB, 0xFF);
    private static final int FIELD_PAD = 30;

    // ── Inner GUI holder ────────────────────────────────────────────────
    private static class PrecomputeProgressGUI {

        final JFrame frame;
        final JProgressBar progressBar;
        final JLabel statusLabel;
        final JTextArea logArea;
        final FieldPanel fieldPanel;
        final InfoPanel infoPanel;

        PrecomputeProgressGUI(int total, double minX, double maxX, double minY, double maxY,
                double targetX, double targetY) {
            frame = new JFrame("Shot Table Precompute");
            progressBar = new JProgressBar(0, total);
            progressBar.setStringPainted(true);
            progressBar.setForeground(ROBOT_COLOR);
            statusLabel = new JLabel(" Starting...");
            statusLabel.setFont(new Font("SansSerif", Font.BOLD, 12));
            logArea = new JTextArea(6, 40);
            logArea.setEditable(false);
            logArea.setBackground(BG_DARK);
            logArea.setForeground(INFO_TEXT);
            logArea.setFont(new Font("Monospaced", Font.PLAIN, 11));

            fieldPanel = new FieldPanel(minX, maxX, minY, maxY, targetX, targetY);
            infoPanel = new InfoPanel();

            JPanel topBar = new JPanel(new BorderLayout(4, 0));
            topBar.add(progressBar, BorderLayout.CENTER);
            topBar.add(statusLabel, BorderLayout.SOUTH);

            JPanel center = new JPanel(new BorderLayout(4, 0));
            center.add(fieldPanel, BorderLayout.CENTER);
            center.add(infoPanel, BorderLayout.EAST);

            frame.setLayout(new BorderLayout(0, 4));
            frame.add(topBar, BorderLayout.NORTH);
            frame.add(center, BorderLayout.CENTER);
            frame.add(new JScrollPane(logArea), BorderLayout.SOUTH);
            frame.setSize(900, 680);
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        }

        void update(int current, int total, int success, int skipped, String details,
                double robotX, double robotY, double shooterX, double shooterY,
                TrajectoryResult result) {
            SwingUtilities.invokeLater(() -> {
                progressBar.setValue(current);
                progressBar.setString(String.format("%d / %d", current, total));
                statusLabel.setText(String.format(" Progress: %d/%d  |  Success: %d  |  Skipped: %d",
                        current, total, success, skipped));

                if (details != null && !details.isEmpty()) {
                    logArea.append(details + "\n");
                    int len = logArea.getDocument().getLength();
                    logArea.setCaretPosition(Math.max(0, Math.min(len, len)));
                }

                // Extract real flight path from the TrajectoryResult
                List<double[]> trajPoints = null;
                if (result != null && result.isSuccess()) {
                    List<Pose3d> flight = result.getFlightPath();
                    if (flight != null && !flight.isEmpty()) {
                        trajPoints = new ArrayList<>(flight.size());
                        for (Pose3d p : flight) {
                            trajPoints.add(new double[]{p.getX(), p.getY()});
                        }
                    }
                }

                // These are not used in this scope; real lists are stored on FieldPanel.

                boolean wasSuccess = result != null && result.isSuccess();
                fieldPanel.pushSample(robotX, robotY, wasSuccess);
                fieldPanel.setRobotAndShooter(robotX, robotY, shooterX, shooterY);
                fieldPanel.setTrajectory(trajPoints);
                fieldPanel.repaint();

                infoPanel.update(result, robotX, robotY, shooterX, shooterY);
            });
        }
    }

    private static class InfoPanel extends JPanel {

        private static final long serialVersionUID = 2L;
        private final JLabel lblStatus, lblPitch, lblVelocity, lblRpm, lblTof,
                lblConfidence, lblRobot, lblShooter, lblDistance;

        InfoPanel() {
            setPreferredSize(new Dimension(220, 0));
            setBackground(INFO_BG);
            setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
            setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

            JLabel header = new JLabel("SAMPLE DATA");
            header.setForeground(INFO_HEADER);
            header.setFont(new Font("SansSerif", Font.BOLD, 13));
            header.setAlignmentX(LEFT_ALIGNMENT);
            add(header);
            add(Box.createVerticalStrut(8));

            lblStatus = makeLabel("Status: —");
            lblPitch = makeLabel("Pitch: —");
            lblVelocity = makeLabel("Velocity: —");
            lblRpm = makeLabel("RPM: —");
            lblTof = makeLabel("ToF: —");
            lblConfidence = makeLabel("Confidence: —");
            lblDistance = makeLabel("Distance: —");
            add(Box.createVerticalStrut(12));
            lblRobot = makeLabel("Robot: —");
            lblShooter = makeLabel("Shooter: —");
        }

        private JLabel makeLabel(String text) {
            JLabel lbl = new JLabel(text);
            lbl.setForeground(INFO_TEXT);
            lbl.setFont(new Font("Monospaced", Font.PLAIN, 11));
            lbl.setAlignmentX(LEFT_ALIGNMENT);
            add(lbl);
            add(Box.createVerticalStrut(3));
            return lbl;
        }

        void update(TrajectoryResult result, double rx, double ry, double sx, double sy) {
            lblRobot.setText(String.format("Robot:   (%.2f, %.2f)", rx, ry));
            lblShooter.setText(String.format("Shooter: (%.2f, %.2f)", sx, sy));
            if (result == null) {
                lblStatus.setText("Status: NO RESULT");
                lblPitch.setText("Pitch: —");
                lblVelocity.setText("Velocity: —");
                lblRpm.setText("RPM: —");
                lblTof.setText("ToF: —");
                lblConfidence.setText("Confidence: —");
                lblDistance.setText("Distance: —");
            } else if (!result.isSuccess()) {
                lblStatus.setText("Status: " + result.getStatus());
                lblPitch.setText("Pitch: —");
                lblVelocity.setText("Velocity: —");
                lblRpm.setText("RPM: —");
                lblTof.setText("ToF: —");
                lblConfidence.setText("Confidence: —");
                lblDistance.setText("Distance: —");
            } else {
                lblStatus.setText("Status: SUCCESS");
                lblPitch.setText(String.format("Pitch: %.2f°", result.getPitchAngleDegrees()));
                lblVelocity.setText(String.format("Velocity: %.2f m/s", result.getRequiredVelocityMps()));
                lblRpm.setText(String.format("RPM: %.0f", result.getRecommendedRpm()));
                lblTof.setText(String.format("ToF: %.3f s", result.getTimeOfFlightSeconds()));
                lblConfidence.setText(String.format("Confidence: %.0f%%", result.getConfidenceScore()));
                lblDistance.setText(String.format("Distance: %.2f m", result.getDistanceToTargetMeters()));
            }
        }
    }

    private static class FieldPanel extends JPanel {

        private static final long serialVersionUID = 1L;
        final double minX, maxX, minY, maxY;
        final double targetX, targetY;
        double robotX, robotY, shooterX, shooterY;
        List<double[]> trajectory;

    // History of sampled points for the heatmap underlay. These are updated from the
    // worker thread while the GUI may paint them; use a synchronized list to avoid
    // ConcurrentModificationExceptions.
    final List<double[]> successPoints = java.util.Collections.synchronizedList(new java.util.ArrayList<>());
    final List<double[]> failPoints = java.util.Collections.synchronizedList(new java.util.ArrayList<>());

        private java.awt.Image backgroundImage = null;

        FieldPanel(double minX, double maxX, double minY, double maxY, double targetX, double targetY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
            this.targetX = targetX;
            this.targetY = targetY;
            setPreferredSize(new Dimension(520, 520));
            setBackground(BG_DARK);

            try {
                java.net.URL imgUrl = FieldPanel.class.getResource("/ca/team4308/absolutelib/math/trajectories/solver/field.png");
                if (imgUrl != null) {
                    backgroundImage = javax.imageio.ImageIO.read(imgUrl);
                }
            } catch (Exception e) {
                // Ignore missing background
            }
        }

        void setRobotAndShooter(double rx, double ry, double sx, double sy) {
            robotX = rx;
            robotY = ry;
            shooterX = sx;
            shooterY = sy;
        }

        void setTrajectory(List<double[]> traj) {
            trajectory = traj;
        }

        void pushSample(double x, double y, boolean success) {
            (success ? successPoints : failPoints).add(new double[]{x, y});
        }

        // ── coordinate mapping ──
        private int toPixelX(double fieldX) {
            return (int) (FIELD_PAD + (fieldX - minX) / (maxX - minX) * (getWidth() - 2 * FIELD_PAD));
        }

        private int toPixelY(double fieldY) {
            return (int) (FIELD_PAD + (fieldY - minY) / (maxY - minY) * (getHeight() - 2 * FIELD_PAD));
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            int w = getWidth(), h = getHeight();

            if (backgroundImage != null) {

                double fieldLengthMeters = 16.541; // 651.22 inches
                double fieldWidthMeters = 8.069;   // 317.677 inches

                double xMinImgMeters = (0 - 524) / (3378.0 - 524.0) * fieldLengthMeters;
                double xMaxImgMeters = (3902 - 524) / (3378.0 - 524.0) * fieldLengthMeters;

                double yMaxImgMeters = (1489 - 0) / (1489.0 - 95.0) * fieldWidthMeters;
                double yMinImgMeters = (1489 - 1509) / (1489.0 - 95.0) * fieldWidthMeters;

                int imgX1 = toPixelX(xMinImgMeters);
                int imgY1 = toPixelY(yMaxImgMeters);
                int imgX2 = toPixelX(xMaxImgMeters);
                int imgY2 = toPixelY(yMinImgMeters);

                int bx = Math.min(imgX1, imgX2);
                int by = Math.min(imgY1, imgY2);
                int bw = Math.abs(imgX2 - imgX1);
                int bh = Math.abs(imgY2 - imgY1);

                g2.drawImage(backgroundImage, bx, by, bw, bh, null);

                g2.setColor(new Color(30, 30, 46, 120));
                g2.fillRect(0, 0, w, h);
            }

            g2.setStroke(new BasicStroke(0.5f));
            g2.setFont(new Font("SansSerif", Font.PLAIN, 9));
            for (double gx = Math.ceil(minX); gx <= maxX; gx += 1.0) {
                int px = toPixelX(gx);
                g2.setColor(GRID_COLOR);
                g2.drawLine(px, FIELD_PAD, px, h - FIELD_PAD);
                g2.setColor(GRID_LABEL_COLOR);
                g2.drawString(String.format("%.0f", gx), px - 4, h - FIELD_PAD + 12);
            }
            for (double gy = Math.ceil(minY); gy <= maxY; gy += 1.0) {
                int py = toPixelY(gy);
                g2.setColor(GRID_COLOR);
                g2.drawLine(FIELD_PAD, py, w - FIELD_PAD, py);
                g2.setColor(GRID_LABEL_COLOR);
                g2.drawString(String.format("%.0f", gy), 2, py + 4);
            }

            // ── Field border ──
            g2.setColor(BORDER_COLOR);
            g2.setStroke(new BasicStroke(1.5f));
            g2.drawRect(FIELD_PAD, FIELD_PAD, w - 2 * FIELD_PAD, h - 2 * FIELD_PAD);

            // ── Sample heatmap dots ──
            synchronized (successPoints) {
                for (double[] pt : successPoints) {
                    int px = toPixelX(pt[0]), py = toPixelY(pt[1]);
                    g2.setColor(SUCCESS_DOT);
                    g2.fill(new Ellipse2D.Double(px - 2, py - 2, 4, 4));
                }
            }
            synchronized (failPoints) {
                for (double[] pt : failPoints) {
                    int px = toPixelX(pt[0]), py = toPixelY(pt[1]);
                    g2.setColor(SKIP_DOT);
                    g2.fill(new Ellipse2D.Double(px - 2, py - 2, 4, 4));
                }
            }

            // ── Target crosshair ──
            int tx = toPixelX(targetX), ty = toPixelY(targetY);
            g2.setColor(TARGET_COLOR);
            g2.setStroke(new BasicStroke(2.0f));
            g2.drawLine(tx - 10, ty, tx + 10, ty);
            g2.drawLine(tx, ty - 10, tx, ty + 10);
            g2.draw(new Ellipse2D.Double(tx - 7, ty - 7, 14, 14));
            g2.setFont(new Font("SansSerif", Font.BOLD, 10));
            g2.drawString("TARGET", tx + 12, ty - 2);

            // ── Trajectory path (real multi-segment) ──
            // Make a local reference to avoid concurrent modification between
            // the worker thread updating the trajectory and the EDT painting.
            List<double[]> traj = trajectory;
            if (traj != null && traj.size() > 1) {
                g2.setStroke(new BasicStroke(2.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                int n = traj.size();
                for (int i = 1; i < n; i++) {
                    float t = (float) i / (n - 1);
                    Color c = blendColor(TRAJ_START, TRAJ_END, t);
                    g2.setColor(c);
                    double[] p0 = traj.get(i - 1);
                    double[] p1 = traj.get(i);
                    g2.draw(new Line2D.Double(
                            toPixelX(p0[0]), toPixelY(p0[1]),
                            toPixelX(p1[0]), toPixelY(p1[1])));
                }
            }

            // ── Robot marker ──
            int rx = toPixelX(robotX), ry = toPixelY(robotY);
            g2.setColor(ROBOT_COLOR);
            g2.fill(new Ellipse2D.Double(rx - 8, ry - 8, 16, 16));
            g2.setColor(ROBOT_COLOR.darker());
            g2.setStroke(new BasicStroke(1.5f));
            g2.draw(new Ellipse2D.Double(rx - 8, ry - 8, 16, 16));
            // direction line toward shooter
            g2.setColor(ROBOT_COLOR);
            int sx2 = toPixelX(shooterX), sy2 = toPixelY(shooterY);
            double dirLen = Math.hypot(sx2 - rx, sy2 - ry);
            if (dirLen > 1) {
                double dx = (sx2 - rx) / dirLen * 14;
                double dy = (sy2 - ry) / dirLen * 14;
                g2.draw(new Line2D.Double(rx, ry, rx + dx, ry + dy));
            }
            g2.setFont(new Font("SansSerif", Font.PLAIN, 9));
            g2.drawString("Robot", rx + 10, ry - 2);

            // ── Shooter marker ──
            g2.setColor(SHOOTER_COLOR);
            g2.fill(new Ellipse2D.Double(sx2 - 5, sy2 - 5, 10, 10));
            g2.drawString("Shooter", sx2 + 8, sy2 - 2);
        }

        private static Color blendColor(Color a, Color b, float t) {
            int r = (int) (a.getRed() + (b.getRed() - a.getRed()) * t);
            int g = (int) (a.getGreen() + (b.getGreen() - a.getGreen()) * t);
            int bl = (int) (a.getBlue() + (b.getBlue() - a.getBlue()) * t);
            return new Color(Math.min(255, Math.max(0, r)),
                    Math.min(255, Math.max(0, g)),
                    Math.min(255, Math.max(0, bl)));
        }
    }

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

            if (config.bounds == null) {
                config.bounds = new ShotTablePrecompute.FieldBounds(0, 16.54, 0, 8.07);
            }
            if (config.outline == null) {
                config.outline = new ShotTablePrecompute.RobotOutline(0.8, 0.8, 0, 0);
            }

            spec.bounds = config.bounds;
            spec.outline = config.outline;
            spec.gridStepMeters = config.precisionMode
                    ? Math.min(config.gridStepMeters, 0.10) : config.gridStepMeters;
            spec.shooterZMeters = config.shooterZMeters;
            spec.targetX = config.targetX;
            spec.targetY = config.targetY;
            spec.targetZ = config.targetZ;
            spec.targetRadiusMeters = config.targetRadiusMeters;
            spec.robotVx = config.robotVx;
            spec.robotVy = config.robotVy;
            spec.includeAirResistance = config.includeAirResistance;
            spec.shotPreference = ShotTablePrecompute.PrecomputeConfig.fromPreference(config.shotPreference);
            spec.maxCandidates = config.precisionMode
                    ? Math.max(config.maxCandidates, 100) : config.maxCandidates;
            spec.minPitchDegrees = config.minPitchDegrees;
            spec.maxPitchDegrees = config.maxPitchDegrees;
            spec.minVelocityMps = config.minVelocityMps;
            spec.maxVelocityMps = config.maxVelocityMps;
            spec.angleStepDegrees = config.precisionMode
                    ? Math.min(config.angleStepDegrees, 0.5) : config.angleStepDegrees;
            spec.minArcHeightMeters = config.minArcHeightMeters;
            spec.preferredArcHeightMeters = config.preferredArcHeightMeters;
            spec.arcBiasStrength = config.arcBiasStrength;
            spec.collisionCheckEnabled = config.collisionCheckEnabled;
            spec.tuningPoints = config.tuningPoints;

            // Robot-specific config from JSON
            spec.flywheelConfig = config.buildFlywheelConfig();
            spec.gamePiece = config.resolveGamePiece();
            spec.solveMode = config.resolveSolveMode();
            spec.solverConfig = config.buildSolverConfig();

            // Half-field: clamp bounds to first half
            // Half-field: compute the half where the target lives
            if (config.halfFieldOnly) {
                double halfX = config.fieldLengthMeters / 2.0;
                if (config.targetX <= halfX) {
                    double clampedMaxX = Math.min(spec.bounds.getMaxX(), halfX);
                    spec.bounds = new ShotTablePrecompute.FieldBounds(
                            spec.bounds.getMinX(), clampedMaxX,
                            spec.bounds.getMinY(), spec.bounds.getMaxY());
                } else {
                    double clampedMinX = Math.max(spec.bounds.getMinX(), halfX);
                    spec.bounds = new ShotTablePrecompute.FieldBounds(
                            clampedMinX, spec.bounds.getMaxX(),
                            spec.bounds.getMinY(), spec.bounds.getMaxY());
                }
            }

            outputPath = Path.of(args.length > 1 ? args[1] : config.outputPath);

            System.out.printf("Config: motor=%s, wheel=%.1f\", gear=%.2f:1, game=%d, mode=%s%s%s%n",
                    config.motorName, config.wheelDiameterInches, config.gearRatio,
                    config.gamePieceYear, config.solveMode,
                    config.precisionMode ? " [PRECISION]" : "",
                    config.halfFieldOnly ? " [HALF-FIELD]" : "");
        } else {
            ShotTablePrecompute.PrecomputeProfile profile
                    = (ShotTablePrecompute.PrecomputeProfile) Class.forName(configArg)
                            .getDeclaredConstructor().newInstance();
            spec = profile.buildSpec();
            outputPath = Path.of(args.length > 1 ? args[1] : spec.outputPath);
        }

        double minX = spec.bounds.getMinX();
        double maxX = spec.bounds.getMaxX();
        double minY = spec.bounds.getMinY();
        double maxY = spec.bounds.getMaxY();
        int totalSamples = (int) Math.ceil(
                ((maxX - minX) / spec.gridStepMeters + 1)
                * ((maxY - minY) / spec.gridStepMeters + 1));

        PrecomputeProgressGUI gui = new PrecomputeProgressGUI(
                totalSamples, minX, maxX, minY, maxY, spec.targetX, spec.targetY);

        ShotTablePrecompute.ShotTable table = ShotTablePrecompute.generateFromSpec(
                spec,
                (current, total, success, skipped, robotX, robotY, shooterX, shooterY, result) -> {
                    String details = null;
                    if (result != null && result.isSuccess()) {
                        details = String.format("#%d  (%.2f,%.2f) → pitch=%.1f° vel=%.1f m/s  ToF=%.3fs",
                                current, robotX, robotY,
                                result.getPitchAngleDegrees(),
                                result.getRequiredVelocityMps(),
                                result.getTimeOfFlightSeconds());
                    } else {
                        String reason = result != null ? result.getStatus().name() : "NULL";
                        details = String.format("#%d  (%.2f,%.2f) → SKIP [%s]",
                                current, robotX, robotY, reason);
                    }
                    gui.update(current, total, success, skipped, details,
                            robotX, robotY, shooterX, shooterY, result);

                    if (current == 1 || current % 25 == 0 || current == total) {
                        System.out.printf("Progress %d/%d (success=%d, skipped=%d)\n",
                                current, total, success, skipped);
                    }
                });
        // Embed metadata into the table for JSON output
        if (configArg.toLowerCase().endsWith(".json")) {
            ShotTablePrecompute.PrecomputeConfig cfg = ShotTablePrecompute.loadConfig(Path.of(configArg));
            table.setExtendedMeta("2.2.0", cfg.gamePieceYear, cfg.motorName,
                    cfg.fieldLengthMeters, cfg.halfFieldOnly);
        } else {
            table.setExtendedMeta("2.2.0",
                    spec.gamePiece != null ? spec.gamePiece.getGameYear() : 2026,
                    "", 16.54, false);
        }

        ShotTablePrecompute.writeJson(outputPath, table);
        System.out.printf("Wrote %d entries to %s (skipped=%d)%n",
                table.getEntries().size(), outputPath, table.getSkippedCount());
    }
}
