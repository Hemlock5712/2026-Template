// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;

/**
 * Compares what the robot did against what replaying the same log produces. Run by the {@code
 * replayCheck} Gradle task - see the run-replay skill.
 *
 * <p>A command-line tool with a {@code main}, not a JUnit test, even though it lives under {@code
 * src/test}. That is deliberate: test code never reaches the robot, and a desktop-only tool has no
 * business in the deployed jar. Leave it here.
 *
 * <p>A replay log holds both: {@code RealOutputs} copied from the recording, and {@code
 * ReplayOutputs} recomputed by this build of the code. With the code unchanged they must agree. If
 * they do not, some value is still coming from hardware instead of the log, and replay is lying.
 */
public final class ReplayCheck {
  private ReplayCheck() {}

  private static final String REAL = "/RealOutputs/";
  private static final String REPLAY = "/ReplayOutputs/";

  // AdvantageKit's own timing telemetry measures elapsed wall-clock, which is meant to differ -
  // replay runs far faster than the robot did.
  private static final String[] IGNORED = {"Logger/", "LoggedRobot/", "Console"};

  // Replayed doubles can land a bit or two off after a round trip through the log; a Rotation2d
  // goes radians -> cos/sin -> atan2 -> radians and comes back ~1e-18 different.
  private static final double RELATIVE_TOLERANCE = 1e-9;
  private static final double ABSOLUTE_TOLERANCE = 1e-12;

  private record Entry(String name, String type, Map<Long, byte[]> values) {}

  public static void main(String[] args) throws Exception {
    if (args.length < 1) {
      System.err.println("usage: ReplayCheck <log>_replay.wpilog");
      System.exit(2);
    }
    DataLogReader reader = new DataLogReader(args[0]);
    if (!reader.isValid()) {
      System.err.println("Not a valid wpilog: " + args[0]);
      System.exit(2);
    }

    Map<Integer, Entry> entries = new HashMap<>();
    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        var start = record.getStartData();
        entries.put(start.entry, new Entry(start.name, start.type, new LinkedHashMap<>()));
      } else if (!record.isControl()) {
        Entry entry = entries.get(record.getEntry());
        if (entry != null) {
          entry.values().put(record.getTimestamp(), record.getRaw());
        }
      }
    }

    Map<String, Entry> byName = new LinkedHashMap<>();
    for (Entry entry : entries.values()) {
      byName.put(entry.name(), entry);
    }

    List<String> failures = new ArrayList<>();
    int compared = 0;
    int skipped = 0;
    for (Entry real : byName.values()) {
      if (!real.name().startsWith(REAL)) {
        continue;
      }
      String key = real.name().substring(REAL.length());
      if (ignored(key)) {
        continue;
      }
      Entry replayed = byName.get(REPLAY + key);
      if (replayed == null) {
        // The recording produced this key and this build never did - a real behaviour change,
        // but also what you see when a key is renamed. Report it, do not fail on it.
        skipped++;
        System.out.println("  (only in recording, not replayed: " + key + ")");
        continue;
      }
      compared++;
      String problem = compare(real, replayed);
      if (problem != null) {
        failures.add("  " + key + ": " + problem);
      }
    }

    System.out.println(
        "Compared " + compared + " output keys (" + skipped + " unmatched, timing keys ignored).");
    if (!failures.isEmpty()) {
      System.out.println("\nReplay does not reproduce the recording:");
      failures.forEach(System.out::println);
      System.out.println(
          "\nSomething is still read from hardware rather than the log. Check that every device\n"
              + "goes through a wrapper in frc/robot/hardware and is read via LoggedHardware.");
      System.exit(1);
    }
    System.out.println("Replay reproduces the recording exactly.");
  }

  private static boolean ignored(String key) {
    for (String prefix : IGNORED) {
      if (key.startsWith(prefix)) {
        return true;
      }
    }
    return false;
  }

  /** Step-hold comparison: at every timestamp either side wrote, the held values must agree. */
  private static String compare(Entry real, Entry replayed) {
    var times = new TreeSet<Long>();
    times.addAll(real.values().keySet());
    times.addAll(replayed.values().keySet());

    byte[] a = null;
    byte[] b = null;
    int mismatches = 0;
    long firstBadTime = 0;
    for (long time : times) {
      a = real.values().getOrDefault(time, a);
      b = replayed.values().getOrDefault(time, b);
      if (a == null || b == null) {
        continue;
      }
      if (!equal(a, b, real.type())) {
        if (mismatches == 0) {
          firstBadTime = time;
        }
        mismatches++;
      }
    }
    if (mismatches == 0) {
      return null;
    }
    return mismatches
        + " of "
        + times.size()
        + " samples differ, first at "
        + String.format("%.3f", firstBadTime / 1e6)
        + "s";
  }

  private static boolean equal(byte[] a, byte[] b, String type) {
    if (a.length != b.length) {
      return false;
    }
    // Doubles - and structs, which WPILib packs as doubles - get a tolerance. Everything else
    // (booleans, integers, strings) has to match exactly.
    boolean numeric =
        type.equals("double") || type.equals("double[]") || type.startsWith("struct:");
    if (!numeric || a.length % Double.BYTES != 0) {
      return java.util.Arrays.equals(a, b);
    }
    var bufA = java.nio.ByteBuffer.wrap(a).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    var bufB = java.nio.ByteBuffer.wrap(b).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    while (bufA.hasRemaining()) {
      double x = bufA.getDouble();
      double y = bufB.getDouble();
      if (Double.compare(x, y) == 0) {
        continue;
      }
      double difference = Math.abs(x - y);
      if (difference > ABSOLUTE_TOLERANCE
          && difference > RELATIVE_TOLERANCE * Math.max(Math.abs(x), Math.abs(y))) {
        return false;
      }
    }
    return true;
  }
}
