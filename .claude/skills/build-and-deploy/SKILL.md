---
name: build-and-deploy
description: How to compile this project from the command line and put code on the robot — the Java 25 JDK requirement, what spotless does on every build, the SystemCore deploy target (not a roboRIO), and what to check before and after deploying. Use when asked to build, compile, check it still compiles, format the code, deploy, push code to the robot, or when a build fails with "invalid source release 25" or a spotless error.
---

# Building and deploying

> File links below are relative to the **repo root**, not to this skill's directory.

For running in simulation instead, see the `run-sim` skill.

## Building

```bash
./gradlew build -Dorg.gradle.java.home=C:/Users/Public/wpilib/2027_alpha5/jdk
```

**The JDK override is mandatory from the command line.** This project is Java 25 source/target, and
Gradle must itself run on a Java 25 JDK. Without it you get:

```
invalid source release: 25
```

That's the single most common build failure here and it means the JDK, not the code. Building from
the WPILib VS Code extension handles this for you.

Useful variants:

| Command | Use |
| --- | --- |
| `./gradlew compileJava` | Fastest check that the code compiles. |
| `./gradlew build` | Compile + spotless + tests. What to run before saying "it builds". |
| `./gradlew clean compileJava` | When you suspect a stale build directory. |
| `./gradlew spotlessApply` | Format only. |
| `./gradlew checkReplaySafety` | Fails on a raw Phoenix device or Phoenix's clock outside `frc/robot/hardware`. Part of `build`. |
| `./gradlew replayCheck` | Replays the newest log and fails if any output differs. Needs a recording first: `./gradlew simulateJavaAgent -PstopAfter=20`. See `run-replay`. |

## Spotless runs on every compile

`build.gradle` wires `JavaCompile.dependsOn 'spotlessApply'`, so **Google Java Format reformats your
files as a side effect of building**. Two consequences:

- Don't hand-format. Write it roughly and let the build fix it.
- A build can leave your working tree modified even when compilation changed nothing. That's
  expected, not a bug.

`spotlessCheck` passes on the pinned JDK — no `-x spotless*` exclusion is needed.

## Deploying to the robot

```bash
./gradlew deploy -Dorg.gradle.java.home=C:/Users/Public/wpilib/2027_alpha5/jdk
```

**The target is SystemCore, not a roboRIO.** The Gradle target is named `systemcore`
(`deploy.targets.systemcore` in [build.gradle](build.gradle)), the code lands in
`/home/systemcore`, and static files from `src/main/deploy/` go to `/home/systemcore/deploy`.
Anything you remember about `/home/lvuser` or `frcUserProgram` does not apply.

Team number comes from [.wpilib/wpilib_preferences.json](.wpilib/wpilib_preferences.json)
(`5712`), which also pins `projectYear` to `2027_alpha5` — the same toolchain the JDK path above
points at.

### Before you deploy

Deploying is the only thing here with real-world consequences — a robot can move. Check:

1. **It builds.** `./gradlew build` clean.
2. **It ran in sim.** See the `run-sim` skill. A clean console is not enough; read the log and
   confirm the robot did what you meant (`log-reading` skill).
3. **Gains are not placeholders.** Several files ship values tuned against the *simulation*, not a
   real robot, and `DriveToTag`'s controllers are all zero. Grep for `TODO` in the files you're
   about to run.
4. **Everyone is clear of the robot** and someone is on the disable button.

### After deploying

- Watch the driver station console for `********** Starting OpMode <name> **********`. OpModes are
  discovered at runtime, so a missing one is silent — see the `add-a-mechanism` skill.
- Logs land on the USB drive at `/U/logs`, not in `./logs` (that's sim). See the `log-reading` skill.

## Alpha-software caveat

WPILib is pinned to `2027.0.0-alpha-6` on purpose, and Phoenix to `26.50.0-alpha-1`. The OpMode and
Commands-v3 APIs still move between alphas. **Don't bump versions casually** — a version bump is a
migration, not an upgrade. See the comment block at the top of [build.gradle](build.gradle).
