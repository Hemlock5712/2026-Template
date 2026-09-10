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
 * OpModeRobot implements the opmode-based robot program framework.
 *
 * <p>The OpModeRobot class is intended to be subclassed by a user creating a robot program.
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
 * <p>Two deliberate differences from {@code OpModeRobot}:
 *
 * <ul>
 *   <li>Everything runs from one periodic callback. {@link #addPeriodic(Runnable, double)} and an
 *       opmode's {@code getCallbacks()} run inline at whole multiples of the loop period instead of
 *       from a priority queue, so one log cycle is always one whole robot cycle and replay is
 *       deterministic. A period that is not a multiple of the loop period is rounded to one.
 *   <li>An opmode's {@code periodic()} runs only between {@code start()} and {@code end()},
 *       matching WPILib main. (Alpha 6 also ran it while the robot was disabled.)
 * </ul>
 */
public abstract class LoggedOpModeRobot extends RobotBase {
  private final ControlWord m_word = new ControlWord();

  private record OpModeFactory(String name, Supplier<OpMode> supplier) {}

  private final Map<Long, OpModeFactory> m_opModes = new HashMap<>();

  /** A user callback and how many loop cycles apart it runs. */
  private record PeriodicCallback(Runnable func, long everyNCycles) {}

  private final List<PeriodicCallback> m_userCallbacks = new ArrayList<>();
  private final List<PeriodicCallback> m_opModeCallbacks = new ArrayList<>();
  private long m_cycleCount;
  private long m_loopStartTimeUs;

  private int m_notifier;
  private final double m_period;
  private final long m_periodUs;
  private final long m_startTimeUs;
  private long m_nextCycleUs;
  private boolean m_useTiming = true;
  private final GcStatsCollector m_gcStatsCollector = new GcStatsCollector();

  // OpMode lifecycle state
  private long m_lastModeId = -1;
  private boolean m_calledDriverStationConnected = false;
  private boolean m_lastEnabledState = false;
  private OpMode m_currentOpMode;
  private boolean m_opModeStarted;
  private final Watchdog m_watchdog;
  private final Alert m_loopOverrunAlert;

  private static void reportAddOpModeError(Class<?> cls, String message) {
    DriverStationErrors.reportError(
        "Error adding OpMode " + cls.getSimpleName() + ": " + message, false);
  }

  private final Optional<Class<? extends UserControls>> m_userControlsBaseClass;
  private UserControls m_userControlsInstance;

  void setUserControlsInstance(UserControls userControlsInstance) {
    if (m_userControlsBaseClass.isEmpty()) {
      throw new IllegalStateException("No UserControls class specified");
    }

    if (!m_userControlsBaseClass.get().isAssignableFrom(userControlsInstance.getClass())) {
      throw new IllegalArgumentException(
          userControlsInstance.getClass().getSimpleName()
              + " is not assignable to "
              + m_userControlsBaseClass.get().getSimpleName());
    }
    m_userControlsInstance = userControlsInstance;
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
    if (m_userControlsBaseClass.isPresent()) {
      ctor = ConstructorMatch.findBestConstructor(cls, getClass(), m_userControlsBaseClass.get());
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
    if (m_userControlsBaseClass.isPresent()) {
      ctor = ConstructorMatch.findBestConstructor(cls, m_userControlsBaseClass.get());
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
      if (m_userControlsInstance != null) {
        return constructor.get().newInstance(this, m_userControlsInstance);
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
    m_opModes.put(id, new OpModeFactory(name, factory));
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
    m_opModes.put(id, new OpModeFactory(name, () -> constructOpModeClass(cls)));
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
      m_opModes.remove(id);
    }
  }

  /** Publishes the operating mode options to the driver station. */
  public void publishOpModes() {
    RobotState.publishOpModes();
  }

  /** Clears all operating mode options and publishes an empty list to the driver station. */
  public void clearOpModes() {
    RobotState.clearOpModes();
    m_opModes.clear();
  }

  /** Default loop period. */
  public static final double DEFAULT_PERIOD = 0.02;

  /** Constructor with default period. */
  @SuppressWarnings("this-escape")
  public LoggedOpModeRobot() {
    this(DEFAULT_PERIOD);
  }

  /**
   * Constructor with specified period.
   *
   * @param period the period at which to run the robot and opmode periodic callbacks.
   */
  @SuppressWarnings("this-escape")
  public LoggedOpModeRobot(double period) {
    m_period = period;
    m_periodUs = (long) (period * 1e6);

    m_notifier = NotifierJNI.createNotifier();
    NotifierJNI.setNotifierName(m_notifier, "LoggedOpModeRobot");

    m_startTimeUs = RobotController.getMonotonicTime();
    m_nextCycleUs = m_startTimeUs + m_periodUs;

    // The logger's install check only knows about LoggedRobot. Remove once Logger.start() accepts
    // LoggedOpModeRobot too.
    Logger.AdvancedHooks.disableRobotBaseCheck();

    m_loopOverrunAlert =
        new Alert("Loop time of \"" + m_period + "\"s overrun", Alert.Level.MEDIUM);
    m_watchdog = new Watchdog(Seconds.of(m_period), () -> m_loopOverrunAlert.set(true));

    // Check to see if we have a DS annotation
    UserControlsInstance userControlsAnnotation =
        getClass().getAnnotation(UserControlsInstance.class);
    if (userControlsAnnotation != null) {
      m_userControlsBaseClass = Optional.of(userControlsAnnotation.value());
    } else {
      m_userControlsBaseClass = Optional.empty();
    }
    // Scan for annotated opmode classes within the derived class's package and subpackages
    addAnnotatedOpModeClasses(getClass().getPackage());
    RobotState.publishOpModes();

    HAL.reportUsage("Framework", "OpModeRobot");
    HAL.reportUsage("LoggingFramework", "AdvantageKit");
  }

  /**
   * Add a callback to run at a specific period.
   *
   * <p>The callback runs inline in the robot loop, at the nearest whole multiple of the loop
   * period. Keeping every callback on the loop's own cadence is what makes one log cycle equal one
   * robot cycle during replay.
   *
   * @param callback The callback to run.
   * @param period The period at which to run the callback, rounded to a multiple of the loop
   *     period.
   */
  public void addPeriodic(Runnable callback, double period) {
    m_userCallbacks.add(new PeriodicCallback(callback, cyclesPer(period)));
  }

  /** Loop cycles per callback period, at least one. */
  private long cyclesPer(double periodSeconds) {
    return Math.max(1, Math.round(periodSeconds / m_period));
  }

  /** Runs the callbacks whose period comes around on this cycle. */
  private void runCallbacks(List<PeriodicCallback> callbacks) {
    for (PeriodicCallback callback : callbacks) {
      if (m_cycleCount % callback.everyNCycles() == 0) {
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
    return m_period;
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
   * updated at the beginning of every periodic callback (including the normal periodic loop).
   *
   * @return Robot running time in microseconds, as of the start of the current periodic function.
   */
  public long getLoopStartTime() {
    return m_loopStartTimeUs;
  }

  /** Main robot loop function. Handles disabled state logic and opmode management. */
  private void loopFunc() {
    m_watchdog.reset();
    boolean enabled = m_word.isEnabled();
    long modeId = m_word.isDSAttached() ? m_word.getOpModeId() : 0;

    if (!m_calledDriverStationConnected && m_word.isDSAttached()) {
      m_calledDriverStationConnected = true;
      driverStationConnected();
      m_watchdog.addEpoch("driverStationConnected()");
    }

    // Handle opmode changes
    if (modeId != m_lastModeId) {
      // Clean up current opmode
      if (m_currentOpMode != null) {
        m_opModeCallbacks.clear();
        m_opModeStarted = false;
        m_currentOpMode.end();
        m_currentOpMode.close();
        m_currentOpMode = null;
      }

      // Set up new opmode
      if (modeId != 0) {
        OpModeFactory factory = m_opModes.get(modeId);
        if (factory != null) {
          // Instantiate the new opmode
          System.out.println("********** Starting OpMode " + factory.name() + " **********");
          m_currentOpMode = factory.supplier().get();
          if (m_currentOpMode != null) {
            // Ensure disabledPeriodic is called at least once
            m_currentOpMode.disabledPeriodic();
            m_watchdog.addEpoch("opMode.disabledPeriodic()");
            // periodic() is called inline below; these are the opmode's extra callbacks.
            for (PeriodicPriorityQueue.Callback callback : m_currentOpMode.getCallbacks()) {
              m_opModeCallbacks.add(
                  new PeriodicCallback(callback.func, cyclesPer(callback.period / 1e6)));
            }
          }
        } else {
          DriverStationErrors.reportError("No OpMode found for mode " + modeId, false);
        }
      }
      m_lastModeId = modeId;
    }

    // Handle enabled state changes
    boolean justCalledDisabledInit = false;
    if (m_lastEnabledState != enabled) {
      if (enabled) {
        // Transitioning to enabled
        disabledExit();
        m_watchdog.addEpoch("disabledExit()");
        if (m_currentOpMode != null) {
          m_currentOpMode.start();
          m_opModeStarted = true;
          m_watchdog.addEpoch("opMode.start()");
        }
      } else {
        // Transitioning to disabled
        if (m_currentOpMode != null && m_lastEnabledState) {
          // Was enabled, now disabled
          m_currentOpMode.end();
          m_opModeStarted = false;
          m_watchdog.addEpoch("opMode.end()");
        }
        disabledInit();
        m_watchdog.addEpoch("disabledInit()");
        justCalledDisabledInit = true;
      }
      m_lastEnabledState = enabled;
    }

    // Call periodic functions based on current state
    if (enabled) {
      if (m_opModeStarted) {
        m_currentOpMode.periodic();
        m_watchdog.addEpoch("opMode.periodic()");
      }
    } else {
      // Only call disabledPeriodic if we didn't just call disabledInit
      if (!justCalledDisabledInit) {
        disabledPeriodic();
        m_watchdog.addEpoch("disabledPeriodic()");
      }

      // Call opmode disabledPeriodic if we have one
      if (m_currentOpMode != null) {
        m_currentOpMode.disabledPeriodic();
        m_watchdog.addEpoch("opMode.disabledPeriodic()");
      }
    }

    // Registered callbacks: the opmode's own, then the robot's.
    runCallbacks(m_opModeCallbacks);
    m_watchdog.addEpoch("opMode callbacks");
    runCallbacks(m_userCallbacks);
    m_watchdog.addEpoch("addPeriodic callbacks");

    // Call nonePeriodic when no opmode is selected
    if (RobotState.getOpModeId() == 0) {
      nonePeriodic();
      m_watchdog.addEpoch("nonePeriodic()");
    }

    // Always call robotPeriodic
    robotPeriodic();
    m_watchdog.addEpoch("robotPeriodic()");

    // Always observe user program state
    DriverStationJNI.observeUserProgram(m_word.getNative());

    SmartDashboard.updateValues();
    m_watchdog.addEpoch("SmartDashboard.updateValues()");

    // Call simulationPeriodic if in simulation
    if (isSimulation()) {
      HAL.simPeriodicBefore();
      simulationPeriodic();
      HAL.simPeriodicAfter();
      m_watchdog.addEpoch("simulationPeriodic()");
    }

    m_watchdog.disable();

    // Flush NetworkTables
    NetworkTableInstance.getDefault().flushLocal();

    // Warn on loop time overruns
    if (m_watchdog.isExpired()) {
      m_watchdog.printEpochs();
    }
  }

  /** Provide an alternate "main loop" via startCompetition(). */
  @Override
  @SuppressWarnings("UnsafeFinalization")
  public void startCompetition() {
    try {
      long initStart = RobotController.getMonotonicTime();
      if (isSimulation()) {
        simulationInit();
      }
      long initEnd = RobotController.getMonotonicTime(); // Includes constructor and opmode scan

      // Register auto logged outputs
      AutoLogOutputManager.addObject(this);

      // Save data from init cycle
      Logger.periodicAfterUser(initEnd - initStart, 0);

      System.out.println("********** Robot program startup complete **********");

      // Tell the DS that the robot is ready to be enabled
      DriverStationBackend.observeUserProgramStarting();

      // Loop forever, one whole robot cycle per pass
      while (true) {
        if (m_useTiming) {
          long currentTimeUs = RobotController.getMonotonicTime();
          if (m_nextCycleUs < currentTimeUs) {
            // Loop overrun, start next cycle immediately
            m_nextCycleUs = currentTimeUs;
          } else {
            NotifierJNI.setNotifierAlarm(m_notifier, m_nextCycleUs, 0, true, true);
            try {
              WPIUtilJNI.waitForObject(m_notifier);
            } catch (InterruptedException ex) {
              Logger.end();
              Thread.currentThread().interrupt();
              break;
            }
          }
          m_nextCycleUs += m_periodUs;
        }

        m_loopStartTimeUs = RobotController.getMonotonicTime();

        long periodicBeforeStart = RobotController.getMonotonicTime();
        Logger.periodicBeforeUser();
        long userCodeStart = RobotController.getMonotonicTime();
        loopFunc();
        long userCodeEnd = RobotController.getMonotonicTime();
        m_cycleCount++;

        // Snapshot the driver station for the next cycle, here rather than at the top of the
        // cycle, because Logger.periodicAfterUser() below is where AdvantageKit saves the DS
        // state: the log then holds the same state the next cycle acts on. Refreshing at the top
        // of a cycle instead is more natural, but a DS change arriving mid-cycle is then written
        // into the entry the cycle already acted on without it, and replay applies that change one
        // cycle early - every enable and opmode transition shifts. The real fix is on the
        // AdvantageKit side: capture the DS in periodicBeforeUser, where replay applies it, so
        // both directions read it at the same point in the cycle.
        DriverStationBackend.refreshData();
        DriverStationBackend.refreshControlWordFromCache(m_word);

        m_gcStatsCollector.update();
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
    m_useTiming = useTiming;
  }

  @Override
  public void close() {
    NotifierJNI.destroyNotifier(m_notifier);
  }

  /** Ends the main loop in startCompetition(). */
  @Override
  public void endCompetition() {
    NotifierJNI.destroyNotifier(m_notifier);
  }

  /** Prints list of epochs added so far and their times. */
  public void printWatchdogEpochs() {
    m_watchdog.printEpochs();
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
