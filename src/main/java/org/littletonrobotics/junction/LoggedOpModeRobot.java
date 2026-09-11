// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Portions copyright (c) FIRST and other WPILib contributors, from
// org.wpilib.framework.OpModeRobot (WPILib BSD license). The opmode registration and lifecycle
// code below is a copy of that class; the loop and the callback scheduling are new.

package org.littletonrobotics.junction;

import static org.wpilib.units.Units.Seconds;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.reflect.Modifier;
import java.net.JarURLConnection;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import org.wpilib.driverstation.Alert;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.UserControls;
import org.wpilib.driverstation.UserControlsInstance;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.ControlWord;
import org.wpilib.hardware.hal.DriverStationJNI;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.NotifierJNI;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.internal.PeriodicPriorityQueue;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.opmode.Utility;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.RobotController;
import org.wpilib.system.Watchdog;
import org.wpilib.util.Color;
import org.wpilib.util.ConstructorMatch;
import org.wpilib.util.WPIUtilJNI;

/**
 * LoggedOpModeRobot implements the opmode-based robot program framework.
 *
 * <p>The LoggedOpModeRobot class is intended to be subclassed by a user creating a robot program,
 * and will call all required AdvantageKit periodic methods.
 *
 * <p>Classes annotated with {@link Autonomous}, {@link Teleop}, and {@link Utility} in the same
 * package or subpackages as the user's subclass are automatically registered as autonomous, teleop,
 * and utility opmodes respectively.
 *
 * <p>Opmodes are constructed when selected on the driver station. While selected and disabled,
 * {@link PeriodicOpMode#disabledPeriodic()} is called. When enabled, {@link PeriodicOpMode#start()}
 * is called once and {@link PeriodicOpMode#periodic()} runs at the rate from {@link #getPeriod()}.
 * On disable or mode switch while enabled, {@link PeriodicOpMode#end()} is called asynchronously
 * and the opmode is then closed and discarded. When no opmode is selected, {@link #nonePeriodic()}
 * is called. {@link #driverStationConnected()} is called once when the DS first connects.
 *
 * <p>This is the AdvantageKit equivalent of WPILib's {@code OpModeRobot}, in the same way {@link
 * LoggedRobot} is the equivalent of {@code TimedRobot}. It is a separate class rather than a
 * subclass because {@code OpModeRobot.startCompetition()} is final and its loop is driven by {@code
 * PeriodicPriorityQueue}, so there is no way to wrap the logging hooks around a cycle or to
 * free-run the loop during replay.
 *
 * <p>Every callback - {@link #addPeriodic(Runnable, double)} and an opmode's own - runs inline on
 * this loop rather than from a priority queue, so one log cycle is always one whole robot cycle and
 * replay is deterministic. A period that is not a multiple of the loop period is rounded to one.
 * The driver station is refreshed at the end of a cycle, for the next one; see the comment in
 * {@link #startCompetition()}.
 */
public abstract class LoggedOpModeRobot extends RobotBase {
  private final ControlWord word = new ControlWord();

  private record OpModeFactory(String name, Supplier<OpMode> supplier) {}

  private final Map<Long, OpModeFactory> opModes = new HashMap<>();

  /** A user callback and how many loop cycles apart it runs. */
  private record PeriodicCallback(Runnable func, long everyNCycles) {}

  private final List<PeriodicCallback> userCallbacks = new ArrayList<>();
  private final List<PeriodicCallback> opModeCallbacks = new ArrayList<>();
  private long cycleCount;
  private long loopStartTimeUs;

  private final int notifier = NotifierJNI.createNotifier();
  private final double periodSecs;
  private final long periodUs;
  private long nextCycleUs = 0;
  private final GcStatsCollector gcStatsCollector = new GcStatsCollector();

  private boolean useTiming = true;

  // OpMode lifecycle state
  private long lastModeId = -1;
  private boolean calledDriverStationConnected = false;
  private boolean lastEnabledState = false;
  private OpMode currentOpMode;
  private boolean opModeStarted;
  private final Watchdog watchdog;
  private final Alert loopOverrunAlert;

  private static void reportAddOpModeError(Class<?> cls, String message) {
    DriverStationErrors.reportError(
        "Error adding OpMode " + cls.getSimpleName() + ": " + message, false);
  }

  private final Optional<Class<? extends UserControls>> userControlsBaseClass;

  // Inert outside org.wpilib.framework: WPILib calls the setter below through OpModeRobot, so
  // nothing can set this. Goes away with UserControls itself in WPILib 2027 alpha 7.
  private UserControls userControlsInstance;

  void setUserControlsInstance(UserControls userControlsInstance) {
    if (userControlsBaseClass.isEmpty()) {
      throw new IllegalStateException("No UserControls class specified");
    }

    if (!userControlsBaseClass.get().isAssignableFrom(userControlsInstance.getClass())) {
      throw new IllegalArgumentException(
          userControlsInstance.getClass().getSimpleName()
              + " is not assignable to "
              + userControlsBaseClass.get().getSimpleName());
    }
    this.userControlsInstance = userControlsInstance;
  }

  /**
   * Find a public constructor to instantiate the opmode. This constructor can have up to 2
   * parameters. The first parameter (if present) must be assignable from this.getClass(). The
   * second parameter (if present) must be assignable from DriverStationBase. If multiple, first
   * sort by most parameters, then by most specific first, then by most specific second.
   */
  private <T> Optional<ConstructorMatch<T>> findOpModeConstructor(Class<T> cls) {
    Optional<ConstructorMatch<T>> ctor;

    // try 2-parameter constructor
    if (userControlsBaseClass.isPresent()) {
      ctor = ConstructorMatch.findBestConstructor(cls, getClass(), userControlsBaseClass.get());
      if (ctor.isPresent()) {
        return ctor;
      }
    }

    // try 1-parameter constructor with RobotBase parameter
    ctor = ConstructorMatch.findBestConstructor(cls, getClass());
    if (ctor.isPresent()) {
      return ctor;
    }

    // try 1-parameter constructor with UserControls parameter
    if (userControlsBaseClass.isPresent()) {
      ctor = ConstructorMatch.findBestConstructor(cls, userControlsBaseClass.get());
      if (ctor.isPresent()) {
        return ctor;
      }
    }

    // try no-parameter constructor
    ctor = ConstructorMatch.findBestConstructor(cls);
    return ctor;
  }

  private <T extends OpMode> T constructOpModeClass(Class<T> cls) {
    Optional<ConstructorMatch<T>> constructor = findOpModeConstructor(cls);
    if (constructor.isEmpty()) {
      DriverStationErrors.reportError(
          "No suitable constructor to instantiate OpMode " + cls.getSimpleName(), true);
      return null;
    }
    try {
      if (userControlsInstance != null) {
        return constructor.get().newInstance(this, userControlsInstance);
      } else {
        return constructor.get().newInstance(this);
      }
    } catch (ReflectiveOperationException e) {
      DriverStationErrors.reportError(
          "Could not instantiate OpMode " + cls.getSimpleName(), e.getStackTrace());
      return null;
    }
  }

  private void checkOpModeClass(Class<?> cls) {
    // the class must be a subclass of OpMode
    if (!OpMode.class.isAssignableFrom(cls)) {
      throw new IllegalArgumentException("not a subclass of OpMode");
    }
    int modifiers = cls.getModifiers();
    // it cannot be abstract
    if (Modifier.isAbstract(modifiers)) {
      throw new IllegalArgumentException("is abstract");
    }
    // it must be public
    if (!Modifier.isPublic(modifiers)) {
      throw new IllegalArgumentException("not public");
    }
    // it must not be a non-static inner class
    if (cls.getEnclosingClass() != null && !Modifier.isStatic(modifiers)) {
      throw new IllegalArgumentException("is a non-static inner class");
    }
    // it must have a public no-arg constructor or a public constructor that accepts this class
    // (or a superclass/interface) as an argument
    if (findOpModeConstructor(cls).isEmpty()) {
      throw new IllegalArgumentException(
          "missing public no-arg constructor or constructor accepting "
              + getClass().getSimpleName());
    }
  }

  /**
   * Adds an opmode using a factory function that creates the opmode. It's necessary to call
   * publishOpModes() to make the added mode visible to the driver station.
   *
   * @param factory factory function to create the opmode
   * @param mode robot mode
   * @param name name of the operating mode
   * @param group group of the operating mode
   * @param description description of the operating mode
   * @param textColor text color, or null for default
   * @param backgroundColor background color, or null for default
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addOpModeFactory(
      Supplier<OpMode> factory,
      RobotMode mode,
      String name,
      String group,
      String description,
      Color textColor,
      Color backgroundColor) {
    long id = RobotState.addOpMode(mode, name, group, description, textColor, backgroundColor);
    opModes.put(id, new OpModeFactory(name, factory));
  }

  /**
   * Adds an opmode using a factory function that creates the opmode. It's necessary to call
   * publishOpModes() to make the added mode visible to the driver station.
   *
   * @param factory factory function to create the opmode
   * @param mode robot mode
   * @param name name of the operating mode
   * @param group group of the operating mode
   * @param description description of the operating mode
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addOpModeFactory(
      Supplier<OpMode> factory, RobotMode mode, String name, String group, String description) {
    addOpModeFactory(factory, mode, name, group, description, null, null);
  }

  /**
   * Adds an opmode using a factory function that creates the opmode. It's necessary to call
   * publishOpModes() to make the added mode visible to the driver station.
   *
   * @param factory factory function to create the opmode
   * @param mode robot mode
   * @param name name of the operating mode
   * @param group group of the operating mode
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addOpModeFactory(
      Supplier<OpMode> factory, RobotMode mode, String name, String group) {
    addOpModeFactory(factory, mode, name, group, "");
  }

  /**
   * Adds an opmode using a factory function that creates the opmode. It's necessary to call
   * publishOpModes() to make the added mode visible to the driver station.
   *
   * @param factory factory function to create the opmode
   * @param mode robot mode
   * @param name name of the operating mode
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addOpModeFactory(Supplier<OpMode> factory, RobotMode mode, String name) {
    addOpModeFactory(factory, mode, name, "");
  }

  /**
   * Adds an opmode for an opmode class. The class must be a public, non-abstract subclass of OpMode
   * with a public constructor that either takes no arguments or accepts a single argument
   * assignable from this robot class type (the latter is preferred; if multiple match, the most
   * specific parameter type is used). It's necessary to call publishOpModes() to make the added
   * mode visible to the driver station.
   *
   * @param cls class to add
   * @param mode robot mode
   * @param name name of the operating mode
   * @param group group of the operating mode
   * @param description description of the operating mode
   * @param textColor text color, or null for default
   * @param backgroundColor background color, or null for default
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addOpMode(
      Class<? extends OpMode> cls,
      RobotMode mode,
      String name,
      String group,
      String description,
      Color textColor,
      Color backgroundColor) {
    checkOpModeClass(cls);
    addOpModeFactory(
        () -> constructOpModeClass(cls),
        mode,
        name,
        group,
        description,
        textColor,
        backgroundColor);
  }

  /**
   * Adds an opmode for an opmode class. The class must be a public, non-abstract subclass of OpMode
   * with a public constructor that either takes no arguments or accepts a single argument
   * assignable from this robot class type (the latter is preferred; if multiple match, the most
   * specific parameter type is used). It's necessary to call publishOpModes() to make the added
   * mode visible to the driver station.
   *
   * @param cls class to add
   * @param mode robot mode
   * @param name name of the operating mode
   * @param group group of the operating mode
   * @param description description of the operating mode
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addOpMode(
      Class<? extends OpMode> cls, RobotMode mode, String name, String group, String description) {
    addOpMode(cls, mode, name, group, description, null, null);
  }

  /**
   * Adds an opmode for an opmode class. The class must be a public, non-abstract subclass of OpMode
   * with a public constructor that either takes no arguments or accepts a single argument
   * assignable from this robot class type (the latter is preferred; if multiple match, the most
   * specific parameter type is used). It's necessary to call publishOpModes() to make the added
   * mode visible to the driver station.
   *
   * @param cls class to add
   * @param mode robot mode
   * @param name name of the operating mode
   * @param group group of the operating mode
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addOpMode(Class<? extends OpMode> cls, RobotMode mode, String name, String group) {
    addOpMode(cls, mode, name, group, "");
  }

  /**
   * Adds an opmode for an opmode class. The class must be a public, non-abstract subclass of OpMode
   * with a public constructor that either takes no arguments or accepts a single argument
   * assignable from this robot class type (the latter is preferred; if multiple match, the most
   * specific parameter type is used). It's necessary to call publishOpModes() to make the added
   * mode visible to the driver station.
   *
   * @param cls class to add
   * @param mode robot mode
   * @param name name of the operating mode
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addOpMode(Class<? extends OpMode> cls, RobotMode mode, String name) {
    addOpMode(cls, mode, name, "");
  }

  private void addOpModeClassImpl(
      Class<? extends OpMode> cls,
      RobotMode mode,
      String name,
      String group,
      String description,
      String textColor,
      String backgroundColor) {
    if (name == null || name.isBlank()) {
      name = cls.getSimpleName();
    }
    Color tColor = textColor.isBlank() ? null : Color.fromString(textColor);
    Color bColor = backgroundColor.isBlank() ? null : Color.fromString(backgroundColor);
    long id = RobotState.addOpMode(mode, name, group, description, tColor, bColor);
    opModes.put(id, new OpModeFactory(name, () -> constructOpModeClass(cls)));
  }

  private void addAnnotatedOpModeImpl(
      Class<? extends OpMode> cls, Autonomous auto, Teleop teleop, Utility utility) {
    checkOpModeClass(cls);

    // add an opmode for each annotation
    if (auto != null) {
      addOpModeClassImpl(
          cls,
          RobotMode.AUTONOMOUS,
          auto.name(),
          auto.group(),
          auto.description(),
          auto.textColor(),
          auto.backgroundColor());
    }
    if (teleop != null) {
      addOpModeClassImpl(
          cls,
          RobotMode.TELEOPERATED,
          teleop.name(),
          teleop.group(),
          teleop.description(),
          teleop.textColor(),
          teleop.backgroundColor());
    }
    if (utility != null) {
      addOpModeClassImpl(
          cls,
          RobotMode.UTILITY,
          utility.name(),
          utility.group(),
          utility.description(),
          utility.textColor(),
          utility.backgroundColor());
    }
  }

  /**
   * Adds an opmode for an opmode class annotated with {@link Autonomous}, {@link Teleop}, or {@link
   * Utility}. The class must be a public, non-abstract subclass of OpMode with a public constructor
   * that either takes no arguments or accepts a single argument assignable from this robot class
   * type (if multiple match, the most specific parameter type is used). It's necessary to call
   * publishOpModes() to make the added mode visible to the driver station.
   *
   * @param cls class to add
   * @throws IllegalArgumentException if class does not meet criteria
   */
  public void addAnnotatedOpMode(Class<? extends OpMode> cls) {
    Autonomous auto = cls.getAnnotation(Autonomous.class);
    Teleop teleop = cls.getAnnotation(Teleop.class);
    Utility utility = cls.getAnnotation(Utility.class);
    if (auto == null && teleop == null && utility == null) {
      throw new IllegalArgumentException("must be annotated with Autonomous, Teleop, or Utility");
    }
    addAnnotatedOpModeImpl(cls, auto, teleop, utility);
  }

  private void addAnnotatedOpModeClass(String name) {
    // trim ".class" from end
    String className = name.replace('/', '.').substring(0, name.length() - 6);
    Class<? extends OpMode> cls;
    try {
      cls = Class.forName(className).asSubclass(OpMode.class);
    } catch (ClassNotFoundException | ClassCastException e) {
      return;
    }
    Autonomous auto = cls.getAnnotation(Autonomous.class);
    Teleop teleop = cls.getAnnotation(Teleop.class);
    Utility utility = cls.getAnnotation(Utility.class);
    if (auto == null && teleop == null && utility == null) {
      return;
    }
    try {
      addAnnotatedOpModeImpl(cls, auto, teleop, utility);
    } catch (IllegalArgumentException e) {
      reportAddOpModeError(cls, e.getMessage());
    }
  }

  private void addAnnotatedOpModeClassesDir(File root, File dir, String packagePath) {
    File[] files = dir.listFiles();
    if (files == null) {
      return;
    }
    for (File file : files) {
      if (file.isDirectory()) {
        addAnnotatedOpModeClassesDir(root, file, packagePath);
      } else if (file.getName().endsWith(".class")) {
        String relPath = root.toPath().relativize(file.toPath()).toString().replace('\\', '/');
        addAnnotatedOpModeClass(packagePath + "." + relPath);
      }
    }
  }

  /**
   * Scans for classes in the specified package and all nested packages that are annotated with
   * {@link Autonomous}, {@link Teleop}, or {@link Utility} and registers them. It's necessary to
   * call publishOpModes() to make the added modes visible to the driver station.
   *
   * @param pkg package to scan
   */
  public void addAnnotatedOpModeClasses(Package pkg) {
    String packageName = pkg.getName();
    String packagePath = packageName.replace('.', '/');
    ClassLoader classLoader = Thread.currentThread().getContextClassLoader();

    try {
      Enumeration<URL> resources = classLoader.getResources(packagePath);
      while (resources.hasMoreElements()) {
        URL resource = resources.nextElement();
        if ("jar".equals(resource.getProtocol())) {
          var connection = resource.openConnection();
          if (!(connection instanceof JarURLConnection jarConnection)) {
            DriverStationErrors.reportError(
                "Error scanning OpModes from "
                    + resource
                    + ": expected JarURLConnection, got "
                    + connection.getClass().getSimpleName(),
                false);
            continue;
          }
          jarConnection.setUseCaches(false);
          try (JarFile jar = jarConnection.getJarFile()) {
            Enumeration<JarEntry> entries = jar.entries();
            while (entries.hasMoreElements()) {
              String name = entries.nextElement().getName();
              if (!name.startsWith(packagePath) || !name.endsWith(".class")) {
                continue;
              }
              addAnnotatedOpModeClass(name);
            }
          }
        } else if ("file".equals(resource.getProtocol())) {
          // Handle .class files in directories
          File dir = new File(resource.toURI());
          if (dir.exists() && dir.isDirectory()) {
            addAnnotatedOpModeClassesDir(dir, dir, packagePath);
          }
        }
      }
    } catch (IOException | URISyntaxException e) {
      e.printStackTrace();
    }
  }

  /**
   * Removes an operating mode option. It's necessary to call publishOpModes() to make the removed
   * mode no longer visible to the driver station.
   *
   * @param mode robot mode
   * @param name name of the operating mode
   */
  public void removeOpMode(RobotMode mode, String name) {
    long id = RobotState.removeOpMode(mode, name);
    if (id != 0) {
      opModes.remove(id);
    }
  }

  /** Publishes the operating mode options to the driver station. */
  public void publishOpModes() {
    RobotState.publishOpModes();
  }

  /** Clears all operating mode options and publishes an empty list to the driver station. */
  public void clearOpModes() {
    RobotState.clearOpModes();
    opModes.clear();
  }

  /** Default loop period. */
  public static final double defaultPeriodSecs = 0.02;

  /** Constructor with default period. */
  @SuppressWarnings("this-escape")
  protected LoggedOpModeRobot() {
    this(defaultPeriodSecs);
  }

  /**
   * Constructor with specified period.
   *
   * @param period the period at which to run the robot and opmode periodic callbacks.
   */
  @SuppressWarnings("this-escape")
  protected LoggedOpModeRobot(double period) {
    periodSecs = period;
    periodUs = (long) (period * 1e6);

    NotifierJNI.setNotifierName(notifier, "LoggedOpModeRobot");

    // Drop once Logger.start()'s install check accepts this class alongside LoggedRobot.
    Logger.AdvancedHooks.disableRobotBaseCheck();

    loopOverrunAlert =
        new Alert("Loop time of \"" + periodSecs + "\"s overrun", Alert.Level.MEDIUM);
    watchdog = new Watchdog(Seconds.of(periodSecs), () -> loopOverrunAlert.set(true));

    // Check to see if we have a DS annotation
    UserControlsInstance userControlsAnnotation =
        getClass().getAnnotation(UserControlsInstance.class);
    if (userControlsAnnotation != null) {
      userControlsBaseClass = Optional.of(userControlsAnnotation.value());
    } else {
      userControlsBaseClass = Optional.empty();
    }
    // Scan for annotated opmode classes within the derived class's package and subpackages
    addAnnotatedOpModeClasses(getClass().getPackage());
    RobotState.publishOpModes();

    HAL.reportUsage("Framework", "AdvantageKit");
    HAL.reportUsage("LoggingFramework", "AdvantageKit");
  }

  /**
   * Add a callback to run at a specific period.
   *
   * <p>The callback runs inline in the robot loop, at the nearest whole multiple of the loop
   * period.
   *
   * @param callback The callback to run.
   * @param period The period at which to run the callback, rounded to a multiple of the loop
   *     period.
   */
  public void addPeriodic(Runnable callback, double period) {
    userCallbacks.add(new PeriodicCallback(callback, cyclesPer(period)));
  }

  /** Loop cycles per callback period, at least one. */
  private long cyclesPer(double periodSeconds) {
    return Math.max(1, Math.round(periodSeconds / periodSecs));
  }

  /** Runs the callbacks whose period comes around on this cycle. */
  private void runCallbacks(List<PeriodicCallback> callbacks) {
    for (PeriodicCallback callback : callbacks) {
      if (cycleCount % callback.everyNCycles() == 0) {
        callback.func().run();
      }
    }
  }

  /**
   * Get the period at which robot and opmode periodic callbacks are run.
   *
   * @return The period at which robot and opmode periodic callbacks are run.
   */
  public double getPeriod() {
    return periodSecs;
  }

  /**
   * Code that needs to know the DS state should go here.
   *
   * <p>Users should override this method for initialization that needs to occur after the DS is
   * connected, such as needing the alliance information.
   */
  public void driverStationConnected() {}

  /** Function called periodically every loop, regardless of enabled state or OpMode selection. */
  public void robotPeriodic() {}

  /** Function called once during robot initialization in simulation. */
  public void simulationInit() {}

  /** Function called periodically in simulation. */
  public void simulationPeriodic() {}

  /** Function called once when the robot becomes disabled. */
  public void disabledInit() {}

  /** Function called periodically while the robot is disabled. */
  public void disabledPeriodic() {}

  /** Function called once when the robot exits disabled state. */
  public void disabledExit() {}

  /**
   * Function called periodically anytime when no opmode is selected, including when the Driver
   * Station is disconnected.
   */
  public void nonePeriodic() {}

  /**
   * Return the system clock time in microseconds for the start of the current periodic loop. This
   * is in the same time base as Timer.getMonotonicTimestamp(), but is stable through a loop. It is
   * updated once per cycle, before any periodic callback runs.
   *
   * @return Robot running time in microseconds, as of the start of the current loop cycle.
   */
  public long getLoopStartTime() {
    return loopStartTimeUs;
  }

  /** Main robot loop function. Handles disabled state logic and opmode management. */
  private void loopFunc() {
    watchdog.reset();
    boolean enabled = word.isEnabled();
    long modeId = word.isDSAttached() ? word.getOpModeId() : 0;

    if (!calledDriverStationConnected && word.isDSAttached()) {
      calledDriverStationConnected = true;
      driverStationConnected();
      watchdog.addEpoch("driverStationConnected()");
    }

    // Handle opmode changes
    if (modeId != lastModeId) {
      // Clean up current opmode
      if (currentOpMode != null) {
        opModeCallbacks.clear();
        opModeStarted = false;
        currentOpMode.end();
        currentOpMode.close();
        currentOpMode = null;
      }

      // Set up new opmode
      if (modeId != 0) {
        OpModeFactory factory = opModes.get(modeId);
        if (factory != null) {
          // Instantiate the new opmode
          System.out.println("********** Starting OpMode " + factory.name() + " **********");
          currentOpMode = factory.supplier().get();
          if (currentOpMode != null) {
            // Ensure disabledPeriodic is called at least once
            currentOpMode.disabledPeriodic();
            watchdog.addEpoch("opMode.disabledPeriodic()");
            // periodic() is called inline below; these are the opmode's extra callbacks.
            for (PeriodicPriorityQueue.Callback callback : currentOpMode.getCallbacks()) {
              opModeCallbacks.add(
                  new PeriodicCallback(callback.func, cyclesPer(callback.period / 1e6)));
            }
          }
        } else {
          DriverStationErrors.reportError("No OpMode found for mode " + modeId, false);
        }
      }
      lastModeId = modeId;
    }

    // Handle enabled state changes
    boolean justCalledDisabledInit = false;
    if (lastEnabledState != enabled) {
      if (enabled) {
        // Transitioning to enabled
        disabledExit();
        watchdog.addEpoch("disabledExit()");
      } else {
        // Transitioning to disabled
        if (currentOpMode != null && lastEnabledState) {
          // Was enabled, now disabled
          currentOpMode.end();
          opModeStarted = false;
          watchdog.addEpoch("opMode.end()");
        }
        disabledInit();
        watchdog.addEpoch("disabledInit()");
        justCalledDisabledInit = true;
      }
      lastEnabledState = enabled;
    }

    // Start the opmode if enabled and not already started. This single check covers both the
    // disabled->enabled transition and an opmode constructed while the robot is already enabled.
    if (enabled && currentOpMode != null && !opModeStarted) {
      currentOpMode.start();
      opModeStarted = true;
      watchdog.addEpoch("opMode.start()");
    }

    // Call periodic functions based on current state
    if (enabled) {
      if (opModeStarted) {
        currentOpMode.periodic();
        watchdog.addEpoch("opMode.periodic()");
      }
    } else {
      // Only call disabledPeriodic if we didn't just call disabledInit
      if (!justCalledDisabledInit) {
        disabledPeriodic();
        watchdog.addEpoch("disabledPeriodic()");
      }

      // Call opmode disabledPeriodic if we have one
      if (currentOpMode != null) {
        currentOpMode.disabledPeriodic();
        watchdog.addEpoch("opMode.disabledPeriodic()");
      }
    }

    // Registered callbacks: the opmode's own, then the robot's.
    runCallbacks(opModeCallbacks);
    watchdog.addEpoch("opMode callbacks");
    runCallbacks(userCallbacks);
    watchdog.addEpoch("addPeriodic callbacks");

    // Call nonePeriodic when no opmode is selected
    if (RobotState.getOpModeId() == 0) {
      nonePeriodic();
      watchdog.addEpoch("nonePeriodic()");
    }

    // Always call robotPeriodic
    robotPeriodic();
    watchdog.addEpoch("robotPeriodic()");

    // Always observe user program state
    DriverStationJNI.observeUserProgram(word.getNative());

    SmartDashboard.updateValues();
    watchdog.addEpoch("SmartDashboard.updateValues()");

    // Call simulationPeriodic if in simulation
    if (isSimulation()) {
      HAL.simPeriodicBefore();
      simulationPeriodic();
      HAL.simPeriodicAfter();
      watchdog.addEpoch("simulationPeriodic()");
    }

    watchdog.disable();

    // Flush NetworkTables
    NetworkTableInstance.getDefault().flushLocal();

    // Warn on loop time overruns
    if (watchdog.isExpired()) {
      watchdog.printEpochs();
    }
  }

  /** Provide an alternate "main loop" via startCompetition(). */
  @Override
  @SuppressWarnings("UnsafeFinalization")
  public void startCompetition() {
    try {
      // Robot init methods
      long initStart = RobotController.getMonotonicTime();
      if (isSimulation()) {
        simulationInit();
      }
      long initEnd = RobotController.getMonotonicTime(); // Includes Robot constructor

      // Register auto logged outputs
      AutoLogOutputManager.addObject(this);

      // Save data from init cycle
      Logger.periodicAfterUser(initEnd - initStart, 0);

      System.out.println("********** Robot program startup complete **********");

      // Tell the DS that the robot is ready to be enabled
      DriverStationBackend.observeUserProgramStarting();

      // Loop forever, calling the appropriate mode-dependent function
      while (true) {
        if (useTiming) {
          long currentTimeUs = RobotController.getMonotonicTime();
          if (nextCycleUs < currentTimeUs) {
            // Loop overrun, start next cycle immediately
            nextCycleUs = currentTimeUs;
          } else {
            // Wait before next cycle
            NotifierJNI.setNotifierAlarm(notifier, nextCycleUs, 0, true, true);
            try {
              WPIUtilJNI.waitForObject(notifier);
            } catch (InterruptedException ex) {
              Logger.end();
              Thread.currentThread().interrupt();
              break;
            }
          }
          nextCycleUs += periodUs;
        }

        long periodicBeforeStart = RobotController.getMonotonicTime();
        loopStartTimeUs = periodicBeforeStart;
        Logger.periodicBeforeUser();
        long userCodeStart = RobotController.getMonotonicTime();
        loopFunc();
        long userCodeEnd = RobotController.getMonotonicTime();
        cycleCount++;

        // Refresh here, immediately before periodicAfterUser() saves the driver station, so the
        // logged control word is the one the next cycle acts on. Refreshing at the top of a cycle
        // instead leaves a whole cycle between read and save, which shifts enable and opmode
        // transitions by one cycle during replay.
        DriverStationBackend.refreshData();
        DriverStationBackend.refreshControlWordFromCache(word);

        gcStatsCollector.update();
        Logger.periodicAfterUser(userCodeEnd - userCodeStart, userCodeStart - periodicBeforeStart);
      }
    } catch (Exception exception) {
      // Exception thrown, log crash information
      StringWriter stringWriter = new StringWriter();
      exception.printStackTrace(new PrintWriter(stringWriter));
      Logger.periodicAfterUser(0, 0, stringWriter.toString());
      Logger.end();
      throw exception;
    }
  }

  /**
   * Sets whether to use standard timing or run as fast as possible.
   *
   * @param useTiming If true, use standard timing. If false, run as fast as possible.
   */
  public void setUseTiming(boolean useTiming) {
    this.useTiming = useTiming;
  }

  @Override
  public void close() {
    NotifierJNI.destroyNotifier(notifier);
    super.close();
  }

  /** Ends the main loop in startCompetition(). */
  @Override
  public void endCompetition() {
    NotifierJNI.destroyNotifier(notifier);
  }

  /** Prints list of epochs added so far and their times. */
  public void printWatchdogEpochs() {
    watchdog.printEpochs();
  }

  private static final class GcStatsCollector {
    private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] lastTimes = new long[gcBeans.size()];
    private final long[] lastCounts = new long[gcBeans.size()];

    public void update() {
      long accumTime = 0;
      long accumCounts = 0;
      for (int i = 0; i < gcBeans.size(); i++) {
        long gcTime = gcBeans.get(i).getCollectionTime();
        long gcCount = gcBeans.get(i).getCollectionCount();
        accumTime += gcTime - lastTimes[i];
        accumCounts += gcCount - lastCounts[i];

        lastTimes[i] = gcTime;
        lastCounts[i] = gcCount;
      }

      Logger.recordOutput("LoggedRobot/GCTimeMS", (double) accumTime);
      Logger.recordOutput("LoggedRobot/GCCounts", (double) accumCounts);
    }
  }
}
