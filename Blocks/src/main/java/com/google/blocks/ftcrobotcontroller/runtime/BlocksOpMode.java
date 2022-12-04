/*
 * Copyright 2016 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.blocks.ftcrobotcontroller.runtime;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.webkit.ConsoleMessage;
import android.webkit.JavascriptInterface;
import android.webkit.WebChromeClient;
import android.webkit.WebView;

import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareType;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil;
import com.google.blocks.ftcrobotcontroller.util.FileUtil;
import com.google.blocks.ftcrobotcontroller.util.Identifier;
import com.google.blocks.ftcrobotcontroller.util.ProjectsUtil;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.EmbeddedControlHubModule;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.internal.opmode.InstanceOpModeManager;
import org.firstinspires.ftc.robotcore.internal.opmode.InstanceOpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

import java.io.IOException;
import java.util.Arrays;
import java.util.Iterator;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * A subclass of {@link LinearOpMode} that loads JavaScript from a file and executes it.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
public final class BlocksOpMode extends LinearOpMode {
  private static final String BLOCK_EXECUTION_ERROR = "Error: Error calling method on NPObject.";
  private static final String LOG_PREFIX = "BlocksOpMode - ";

  private static final AtomicReference<RuntimeException> fatalExceptionHolder = new AtomicReference<RuntimeException>();
  private static final AtomicReference<String> fatalErrorMessageHolder = new AtomicReference<String>();

  @SuppressLint("StaticFieldLeak")
  private static Activity activity;
  @SuppressLint("StaticFieldLeak")
  private static WebView webView;
  private static final AtomicReference<String> nameOfOpModeLoadedIntoWebView = new AtomicReference<String>();
  // Visible for testing.
  static final Map<String, Access> javascriptInterfaces = new ConcurrentHashMap<String, Access>();
  private final String project;
  private final String logPrefix;
  private final AtomicLong interruptedTime = new AtomicLong();

  private volatile BlockType currentBlockType;
  private volatile boolean currentBlockFinished;
  private volatile String currentBlockFirstName;
  private volatile String currentBlockLastName;
  private volatile Thread javaBridgeThread;

  private volatile boolean forceStopped = false;
  private volatile boolean wasTerminated = false;
  private CameraName switchableCamera;

  /**
   * Instantiates a BlocksOpMode that loads JavaScript from a file and executes it when the op mode
   * is run.
   *
   * @param project the name of the project.
   */
  // Visible for testing
  BlocksOpMode(String project) {
    super();
    this.project = project;
    logPrefix = LOG_PREFIX + "\"" + project + "\" - ";
  }

  private String getLogPrefix() {
    Thread thread = Thread.currentThread();
    return logPrefix + thread.getThreadGroup().getName() + "/" + thread.getName() + " - ";
  }

  void startBlockExecution(BlockType blockType, String blockFirstName, String blockLastName) {
    currentBlockType = blockType;
    currentBlockFirstName = blockFirstName;
    currentBlockLastName = blockLastName;
    currentBlockFinished = false;
    checkIfStopRequested();
  }

  void endBlockExecution() {
    if (fatalExceptionHolder.get() == null) {
      currentBlockFinished = true;
    }
  }

  String getFullBlockLabel() {
    switch (currentBlockType) {
      default:
        return "to runOpmode";
      case SPECIAL:
        return currentBlockFirstName + currentBlockLastName;
      case EVENT:
        return "to " +  currentBlockFirstName + currentBlockLastName;
      case CREATE:
        return "new " + currentBlockFirstName;
      case SETTER:
        return "set " + currentBlockFirstName + currentBlockLastName + " to";
      case GETTER:
        return currentBlockFirstName + currentBlockLastName;
      case FUNCTION:
        return "call " + currentBlockFirstName + currentBlockLastName;
    }
  }

  void handleFatalException(Throwable e) {
    String errorMessage = e.getClass().getSimpleName() + (e.getMessage() != null ? " - " + e.getMessage() : "");
    RuntimeException re = new RuntimeException(
        "Fatal error occurred while executing the block labeled \"" + getFullBlockLabel() + "\". " +
        errorMessage, e);
    fatalExceptionHolder.set(re);
    throw re; // This will cause the opmode to stop.
  }

  private void checkIfStopRequested() {
    if (interruptedTime.get() != 0L &&
        isStopRequested() &&
        System.currentTimeMillis() - interruptedTime.get() >= /* bite BEFORE main watchdog*/msStuckDetectStop-100) {
      RobotLog.i(getLogPrefix() + "checkIfStopRequested - about to stop opmode by throwing RuntimeException");
      forceStopped = true;
      throw new RuntimeException("Stopping opmode " + project + " by force.");
    }
  }

  void waitForStartForBlocks() {
    // Because this method is executed on the Java Bridge thread, it is not interrupted when stop
    // is called. To fix this, we repeatedly wait 100ms and check isStarted.
    RobotLog.i(getLogPrefix() + "waitForStartForBlocks - start");
    try {
      while (!isStartedForBlocks()) {
        synchronized (this) {
          try {
            this.wait(100);
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
          }
        }
      }
    } finally {
      RobotLog.i(getLogPrefix() + "waitForStartForBlocks - end");
    }
  }

  void sleepForBlocks(long millis) {
    // Because this method is executed on the Java Bridge thread, it is not interrupted when stop
    // is called. To fix this, we repeatedly sleep 100ms and check isInterrupted.
    RobotLog.i(getLogPrefix() + "sleepForBlocks - start");
    try {
      long endTime = System.currentTimeMillis() + millis;
      while (!isInterrupted()) {
        long chunk = Math.min(100L, endTime - System.currentTimeMillis());
        if (chunk <= 0) {
          break;
        }
        sleep(chunk);
      }
    } finally {
      RobotLog.i(getLogPrefix() + "sleepForBlocks - end");
    }
  }

  private boolean isInterrupted() {
    return interruptedTime.get() != 0L;
  }

  boolean isStartedForBlocks() {
    return super.isStarted() || isInterrupted();
  }

  boolean isStopRequestedForBlocks() {
    return super.isStopRequested() || isInterrupted();
  }

  void terminateOpModeNowForBlocks() {
    wasTerminated = true;
    super.terminateOpModeNow();
  }

  @Override
  public void runOpMode() {
    RobotLog.i(getLogPrefix() + "runOpMode - start");
    cleanUpPreviousBlocksOpMode();

    BlocksOpModeCompanion.opMode = this;
    BlocksOpModeCompanion.linearOpMode = this;
    BlocksOpModeCompanion.hardwareMap = hardwareMap;
    BlocksOpModeCompanion.telemetry = telemetry;
    BlocksOpModeCompanion.gamepad1 = gamepad1;
    BlocksOpModeCompanion.gamepad2 = gamepad2;

    try {
      fatalExceptionHolder.set(null);
      fatalErrorMessageHolder.set(null);

      currentBlockType = BlockType.EVENT;
      currentBlockFirstName = "";
      currentBlockLastName = "runOpMode";

      boolean interrupted = false;
      interruptedTime.set(0L);

      final AtomicBoolean scriptFinished = new AtomicBoolean();
      final Object scriptFinishedLock = new Object();

      final BlocksOpModeAccess blocksOpModeAccess = new BlocksOpModeAccess(
          Identifier.BLOCKS_OP_MODE.identifierForJavaScript, scriptFinishedLock, scriptFinished);

      javascriptInterfaces.put(
          Identifier.BLOCKS_OP_MODE.identifierForJavaScript, blocksOpModeAccess);

      // Start running the user's op mode blocks by calling loadScript on the UI thread.
      // Execution of the script is done by the WebView component, which uses the Java Bridge
      // thread to call into java.

      AppUtil appUtil = AppUtil.getInstance();

      synchronized (scriptFinishedLock) {
        appUtil.runOnUiThread(new Runnable() {
          @Override
          public void run() {
            try {
              RobotLog.i(getLogPrefix() + "run1 - before loadScript");
              loadScript();
              RobotLog.i(getLogPrefix() + "run1 - after loadScript");
            } catch (Exception e) {
              RobotLog.e(getLogPrefix() + "run1 - caught " + e);
              // The exception may not have a stacktrace, so we check before calling
              // RobotLog.logStackTrace.
              if (e.getStackTrace() != null) {
                RobotLog.logStackTrace(e);
              }
            }
          }
        });

        // This thread (the thread executing BlocksOpMode.runOpMode) waits for the script to finish
        // When the script finishes, it calls BlocksOpModeAccess.scriptFinished() (on the Java
        // Bridge thread), which will set scriptFinished to true and call
        // scriptFinishedLock.notifyAll(). At that point, the scriptFinished.wait() call below
        // finish, allowing this thread to continue running.

        // If the stop button is pressed, the scriptFinished.wait() call below will be interrrupted
        // and this thread will catch InterruptedException. The script will continue to run and
        // this thread will continue to wait until scriptFinished is set. However, all calls from
        // javascript into java call startBlockExecution. startBlockExecution calls
        // checkIfStopRequested, which will throw an exception if the elapsed time since catching
        // the InterruptedException exceeds msStuckDetectStop. The exception will cause the script
        // to stop immediately, set scriptFinished to true and call scriptFinished.notifyAll().

        RobotLog.i(getLogPrefix() + "runOpMode - before while !scriptFinished loop");
        while (!scriptFinished.get()) {
          try {
            scriptFinishedLock.wait();
          } catch (InterruptedException e) {
            RobotLog.e(getLogPrefix() + "runOpMode - caught InterruptedException during scriptFinishedLock.wait");
            interrupted = true;
            interruptedTime.set(System.currentTimeMillis());
            /* Non-blocks specific code running on the Java bridge thread is unable to call isStopRequestedForBlocks.
               For that code, it's important to interrupt the thread, so that the normal interrupt handling code runs. */
            if (javaBridgeThread != null) {
              javaBridgeThread.interrupt();
            }
          }
        }
        RobotLog.i(getLogPrefix() + "runOpMode - after while !scriptFinished loop");
      }

      clearSwitchableCamera();

      // Clean up the WebView component by calling clearScript on the UI thread.
      appUtil.runOnUiThread(new Runnable() {
        @Override
        public void run() {
          try {
            RobotLog.i(getLogPrefix() + "run2 - before clearScript");
            clearScript();
            RobotLog.i(getLogPrefix() + "run2 - after clearScript");
          } catch (Exception e) {
            RobotLog.e(getLogPrefix() + "run2 - caught " + e);
            // The exception may not have a stacktrace, so we check before calling
            // RobotLog.logStackTrace.
            if (e.getStackTrace() != null) {
              RobotLog.logStackTrace(e);
            }
          }
        }
      });

      // If an InterruptedException was caught, call Thread.currentThread().interrupt() to set
      // the interrupted status.

      if (interrupted) {
        Thread.currentThread().interrupt();
      }

      // If there was an exception, throw it now.
      RuntimeException fatalException = fatalExceptionHolder.getAndSet(null);
      if (fatalException != null) {
        throw fatalException;
      }

      // If there was a fatal error in the WebView component, set the global error message.
      String fatalErrorMessage = fatalErrorMessageHolder.getAndSet(null);
      if (fatalErrorMessage != null) {
        RobotLog.setGlobalErrorMsg(fatalErrorMessage);
      }
    } finally {
      long interruptedTime = this.interruptedTime.get();
      if (interruptedTime != 0L) {
        RobotLog.i(getLogPrefix() + "runOpMode - end - " +
            (System.currentTimeMillis() - interruptedTime) + "ms after InterruptedException");
      } else {
        RobotLog.i(getLogPrefix() + "runOpMode - end - no InterruptedException");
      }
      BlocksOpModeCompanion.opMode = null;
      BlocksOpModeCompanion.linearOpMode = null;
    }
  }

  private void cleanUpPreviousBlocksOpMode() {
    String name = nameOfOpModeLoadedIntoWebView.get();
    if (name != null) {
      RobotLog.w(getLogPrefix() + "cleanUpPreviousBlocksOpMode - Warning: The Blocks runtime system is still loaded " +
          "with the Blocks op mode named " + name + ".");
      RobotLog.w(getLogPrefix() + "cleanUpPreviousBlocksOpMode - Trying to clean up now.");
      AppUtil.getInstance().synchronousRunOnUiThread(new Runnable() {
        @Override
        public void run() {
          try {
            RobotLog.w(getLogPrefix() + "cleanUpPreviousBlocksOpMode run - before clearScript");
            clearScript();
            RobotLog.w(getLogPrefix() + "cleanUpPreviousBlocksOpMode run - after clearScript");
          } catch (Exception e) {
            RobotLog.e(getLogPrefix() + "cleanUpPreviousBlocksOpMode run - caught " + e);
            // The exception may not have a stacktrace, so we check before calling
            // RobotLog.logStackTrace.
            if (e.getStackTrace() != null) {
              RobotLog.logStackTrace(e);
            }
          }
        }
      });
      if (nameOfOpModeLoadedIntoWebView.get() != null) {
        RobotLog.w(getLogPrefix() + "cleanUpPreviousBlocksOpMode - Clean up was successful.");
      } else {
        RobotLog.e(getLogPrefix() + "cleanUpPreviousBlocksOpMode - Error: Clean up failed.");
        throw new RuntimeException(
            "Unable to start running the Blocks op mode named " + project + ". The Blocks runtime " +
            "system is still loaded with the previous Blocks op mode named " + name + ". " +
            "Please restart the Robot Controller app.");
      }
    }
  }

  @SuppressLint("JavascriptInterface")
  private void addJavascriptInterfaces(HardwareItemMap hardwareItemMap, Set<String> identifiersUsed) {
    addJavascriptInterfacesForIdentifiers();
    addJavascriptInterfacesForHardware(hardwareItemMap, identifiersUsed);

    for (Map.Entry<String, Access> entry : javascriptInterfaces.entrySet()) {
      String identifier = entry.getKey();
      Access access = entry.getValue();
      webView.addJavascriptInterface(access, identifier);
    }
  }

  // Visible for testing.
  void addJavascriptInterfacesForIdentifiers() {
    javascriptInterfaces.put(Identifier.ACCELERATION.identifierForJavaScript,
        new AccelerationAccess(this, Identifier.ACCELERATION.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.ANDROID_ACCELEROMETER.identifierForJavaScript,
        new AndroidAccelerometerAccess(this, Identifier.ANDROID_ACCELEROMETER.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.ANDROID_GYROSCOPE.identifierForJavaScript,
        new AndroidGyroscopeAccess(this, Identifier.ANDROID_GYROSCOPE.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.ANDROID_ORIENTATION.identifierForJavaScript,
        new AndroidOrientationAccess(this, Identifier.ANDROID_ORIENTATION.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.ANDROID_SOUND_POOL.identifierForJavaScript,
        new AndroidSoundPoolAccess(this, Identifier.ANDROID_SOUND_POOL.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.ANDROID_TEXT_TO_SPEECH.identifierForJavaScript,
        new AndroidTextToSpeechAccess(this, Identifier.ANDROID_TEXT_TO_SPEECH.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.ANGULAR_VELOCITY.identifierForJavaScript,
        new AngularVelocityAccess(this, Identifier.ANGULAR_VELOCITY.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.BLINKIN_PATTERN.identifierForJavaScript,
        new BlinkinPatternAccess(this, Identifier.BLINKIN_PATTERN.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.BNO055IMU_PARAMETERS.identifierForJavaScript,
        new BNO055IMUParametersAccess(this, Identifier.BNO055IMU_PARAMETERS.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.COLOR.identifierForJavaScript,
        new ColorAccess(this, Identifier.COLOR.identifierForJavaScript, activity));
    javascriptInterfaces.put(Identifier.DBG_LOG.identifierForJavaScript,
        new DbgLogAccess(this, Identifier.DBG_LOG.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.ELAPSED_TIME.identifierForJavaScript,
        new ElapsedTimeAccess(this, Identifier.ELAPSED_TIME.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.GAMEPAD_1.identifierForJavaScript,
        new GamepadAccess(this, Identifier.GAMEPAD_1.identifierForJavaScript, gamepad1));
    javascriptInterfaces.put(Identifier.GAMEPAD_2.identifierForJavaScript,
        new GamepadAccess(this, Identifier.GAMEPAD_2.identifierForJavaScript, gamepad2));
    javascriptInterfaces.put(Identifier.LED_EFFECT.identifierForJavaScript,
        new LedEffectAccess(this, Identifier.LED_EFFECT.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.LINEAR_OP_MODE.identifierForJavaScript,
        new LinearOpModeAccess(this, Identifier.LINEAR_OP_MODE.identifierForJavaScript, project));
    javascriptInterfaces.put(Identifier.MAGNETIC_FLUX.identifierForJavaScript,
        new MagneticFluxAccess(this, Identifier.MAGNETIC_FLUX.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.MATRIX_F.identifierForJavaScript,
        new MatrixFAccess(this, Identifier.MATRIX_F.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.MISC.identifierForJavaScript,
        new MiscAccess(this, Identifier.MISC.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.NAVIGATION.identifierForJavaScript,
        new NavigationAccess(this, Identifier.NAVIGATION.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.OPEN_GL_MATRIX.identifierForJavaScript,
        new OpenGLMatrixAccess(this, Identifier.OPEN_GL_MATRIX.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.ORIENTATION.identifierForJavaScript,
        new OrientationAccess(this, Identifier.ORIENTATION.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.PIDF_COEFFICIENTS.identifierForJavaScript,
        new PIDFCoefficientsAccess(this, Identifier.PIDF_COEFFICIENTS.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.POSITION.identifierForJavaScript,
        new PositionAccess(this, Identifier.POSITION.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.QUATERNION.identifierForJavaScript,
        new QuaternionAccess(this, Identifier.QUATERNION.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.RANGE.identifierForJavaScript,
        new RangeAccess(this, Identifier.RANGE.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.RUMBLE_EFFECT.identifierForJavaScript,
        new RumbleEffectAccess(this, Identifier.RUMBLE_EFFECT.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.SYSTEM.identifierForJavaScript,
        new SystemAccess(this, Identifier.SYSTEM.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.TELEMETRY.identifierForJavaScript,
        new TelemetryAccess(this, Identifier.TELEMETRY.identifierForJavaScript, telemetry));
    javascriptInterfaces.put(Identifier.TEMPERATURE.identifierForJavaScript,
        new TemperatureAccess(this, Identifier.TEMPERATURE.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.TFOD.identifierForJavaScript,
        new TfodAccess(this, Identifier.TFOD.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.TFOD_CURRENT_GAME.identifierForJavaScript,
        new TfodCurrentGameAccess(this, Identifier.TFOD_CURRENT_GAME.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.TFOD_CUSTOM_MODEL.identifierForJavaScript,
        new TfodCustomModelAccess(this, Identifier.TFOD_CUSTOM_MODEL.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.TFOD_ROVER_RUCKUS.identifierForJavaScript,
        new TfodRoverRuckusAccess(this, Identifier.TFOD_ROVER_RUCKUS.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.TFOD_SKY_STONE.identifierForJavaScript,
        new TfodSkyStoneAccess(this, Identifier.TFOD_SKY_STONE.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.VECTOR_F.identifierForJavaScript,
        new VectorFAccess(this, Identifier.VECTOR_F.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.VELOCITY.identifierForJavaScript,
        new VelocityAccess(this, Identifier.VELOCITY.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.VUFORIA_CURRENT_GAME.identifierForJavaScript,
        new VuforiaCurrentGameAccess(this, Identifier.VUFORIA_CURRENT_GAME.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.VUFORIA_RELIC_RECOVERY.identifierForJavaScript,
        new VuforiaRelicRecoveryAccess(this, Identifier.VUFORIA_RELIC_RECOVERY.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.VUFORIA_ROVER_RUCKUS.identifierForJavaScript,
        new VuforiaRoverRuckusAccess(this, Identifier.VUFORIA_ROVER_RUCKUS.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.VUFORIA_SKY_STONE.identifierForJavaScript,
        new VuforiaSkyStoneAccess(this, Identifier.VUFORIA_SKY_STONE.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.VUFORIA_LOCALIZER.identifierForJavaScript,
        new VuforiaLocalizerAccess(this, Identifier.VUFORIA_LOCALIZER.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.VUFORIA_LOCALIZER_PARAMETERS.identifierForJavaScript,
        new VuforiaLocalizerParametersAccess(this, Identifier.VUFORIA_LOCALIZER_PARAMETERS.identifierForJavaScript, activity, hardwareMap));
    javascriptInterfaces.put(Identifier.VUFORIA_TRACKABLE.identifierForJavaScript,
        new VuforiaTrackableAccess(this, Identifier.VUFORIA_TRACKABLE.identifierForJavaScript));
    javascriptInterfaces.put(Identifier.VUFORIA_TRACKABLE_DEFAULT_LISTENER.identifierForJavaScript,
        new VuforiaTrackableDefaultListenerAccess(this, Identifier.VUFORIA_TRACKABLE_DEFAULT_LISTENER.identifierForJavaScript, hardwareMap));
    javascriptInterfaces.put(Identifier.VUFORIA_TRACKABLES.identifierForJavaScript,
        new VuforiaTrackablesAccess(this, Identifier.VUFORIA_TRACKABLES.identifierForJavaScript));
  }

  private void addJavascriptInterfacesForHardware(HardwareItemMap hardwareItemMap, Set<String> identifiersUsed) {
    for (HardwareType hardwareType : HardwareType.values()) {
      if (hardwareItemMap.contains(hardwareType)) {
        for (HardwareItem hardwareItem : hardwareItemMap.getHardwareItems(hardwareType)) {
          // Don't instantiate the HardwareAccess instance if the identifier isn't used in the
          // blocks opmode.
          if (identifiersUsed != null && !identifiersUsed.contains(hardwareItem.identifier)) {
            RobotLog.i(getLogPrefix() + "Skipping hardware device named \"" +
                hardwareItem.deviceName + "\". It isn't used in this blocks opmode.");
            continue;
          }
          if (javascriptInterfaces.containsKey(hardwareItem.identifier)) {
            RobotLog.w(getLogPrefix() + "There is already a JavascriptInterface for identifier \"" +
                hardwareItem.identifier + "\". Ignoring hardware type " + hardwareType + ".");
            continue;
          }
          Access access =
              HardwareAccess.newHardwareAccess(this, hardwareType, hardwareMap, hardwareItem);
          if (access != null) {
            javascriptInterfaces.put(hardwareItem.identifier, access);
          }
        }
      }
    }
  }

  private void removeJavascriptInterfaces() {
    Iterator<Map.Entry<String, Access>> it = javascriptInterfaces.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, Access> entry = it.next();
      String identifier = entry.getKey();
      Access access = entry.getValue();
      webView.removeJavascriptInterface(identifier);
      access.close();
      it.remove();
    }
  }

  CameraName getSwitchableCamera() {
    if (switchableCamera == null) {
      switchableCamera = VuforiaBase.getSwitchableCamera(hardwareMap);
    }
    return switchableCamera;
  }

  private void clearSwitchableCamera() {
    switchableCamera = null;
  }

  private class BlocksOpModeAccess extends Access {
    private final Object scriptFinishedLock;
    private final AtomicBoolean scriptFinished;

    private BlocksOpModeAccess(String identifier, Object scriptFinishedLock, AtomicBoolean scriptFinished) {
      super(BlocksOpMode.this, identifier, "");
      this.scriptFinishedLock = scriptFinishedLock;
      this.scriptFinished = scriptFinished;
    }

    @SuppressWarnings("unused")
    @JavascriptInterface
    public void scriptStarting() {
      RobotLog.i(getLogPrefix() + "scriptStarting");
      /* Clear the interrupt flag on this thread (the JavaBridge thread), which may have been set
         during a previous Blocks Op Mode run.

         Note that this thread is tied to the WebView, which is why it is reused for all Blocks
         Op Modes.

         We clear the interrupt flag by calling the (normally undesirable) method Thread.interrupted().
      */

      //noinspection ResultOfMethodCallIgnored
      Thread.interrupted();
      javaBridgeThread = Thread.currentThread();
    }

    @SuppressWarnings("unused")
    @JavascriptInterface
    public void caughtException(String message, String currentBlockLabel) {
      if (wasTerminated) {
        return;
      }

      if (message != null) {
        // If a hardware device is used in blocks, but has been removed (or renamed) in the
        // configuration, the message is like "ReferenceError: left_drive is not defined".
        if (message.startsWith("ReferenceError: ") && message.endsWith(" is not defined")) {
          String missingIdentifier = message.substring(16, message.length() - 15);

          String errorMessage = "Could not find identifier: " + missingIdentifier;

          // See if we can improve the error message.
          String missingHardwareDeviceName = missingIdentifierToHardwareDeviceName(missingIdentifier);
          if (missingHardwareDeviceName != null) {
            errorMessage = "Could not find hardware device: " + missingHardwareDeviceName;

            if (missingIdentifier.endsWith(HardwareType.BNO055IMU.identifierSuffixForJavaScript)) {
              String wrongImuErrorMessage = getWrongImuErrorMessage();
              if (wrongImuErrorMessage != null) {
                errorMessage += "\n\n" + wrongImuErrorMessage;
              }
            }
          }

          fatalErrorMessageHolder.compareAndSet(null, errorMessage);
          return;
        }

        if (forceStopped)
        {
           AppUtil.getInstance().showAlertDialog(UILocation.BOTH, "OpMode Force-Stopped",
               "User OpMode was stuck in stop(), but was able to be force stopped without " +
               "restarting the app. Please make sure you are calling opModeInInit() or " +
               "opModeIsActive() in any loops!");

           // Get out of dodge so we don't force a restart by setting a global error
           return;
        }

        // An exception occured while the blocks opmode was executing.
        // If the currentBlockLabel parameter is not empty, it is the label of the block that caused the exception.
        if (currentBlockLabel != null && !currentBlockLabel.isEmpty()) {
          fatalErrorMessageHolder.compareAndSet(null,
              "Fatal error occurred while executing the block labeled \"" +
              currentBlockLabel + "\". " + message);
        } else {
          // Otherwise, we use the label of the last block whose java code called
          // startBlockExecution.
          if (currentBlockFinished) {
            fatalErrorMessageHolder.compareAndSet(null,
                "Fatal error occurred after executing the block labeled \"" +
                getFullBlockLabel() + "\". " + message);
          } else {
            fatalErrorMessageHolder.compareAndSet(null,
                "Fatal error occurred while executing the block labeled \"" +
                getFullBlockLabel() + "\". " + message);
          }
        }
      }

      RobotLog.e(getLogPrefix() + "caughtException - message is " + message);
    }

    private String getWrongImuErrorMessage() {
      Context context = AppUtil.getDefContext();
      LynxModule controlHub = EmbeddedControlHubModule.get();
      LynxI2cDeviceSynch tempImuI2cClient = LynxFirmwareVersionManager.createLynxI2cDeviceSynch(context, controlHub, 0);
      try {
        if (BHI260IMU.imuIsPresent(tempImuI2cClient)) {
          return "You attempted to use a BNO055 IMU on a Control Hub that contains a BHI260AP IMU. " +
              "You need to migrate your IMU code to the new driver when it becomes available in version 8.1 of the FTC Robot Controller app.";
        }
      } finally {
        tempImuI2cClient.close();
      }
      return null;
    }

    @SuppressWarnings("unused")
    @JavascriptInterface
    public void scriptFinished() {
      RobotLog.i(getLogPrefix() + "scriptFinished");
      synchronized (scriptFinishedLock) {
        scriptFinished.set(true);
        scriptFinishedLock.notifyAll();
      }
    }
  }

  private static String missingIdentifierToHardwareDeviceName(String identifier) {
    for (HardwareType hardwareType : HardwareType.values()) {
      if (identifier.endsWith(hardwareType.identifierSuffixForJavaScript)) {
        return identifier.substring(0, identifier.length() - hardwareType.identifierSuffixForJavaScript.length());
      }
    }
    return null;
  }

  private void loadScript() throws IOException {
    RobotLog.i(getLogPrefix() + "loadScript - WebView user agent is \"" + webView.getSettings().getUserAgentString() + "\"");
    nameOfOpModeLoadedIntoWebView.set(project);
    HardwareItemMap hardwareItemMap = HardwareItemMap.newHardwareItemMap(hardwareMap);

    // Check if the javascript begins with a comment telling us what the identifiers are.
    Set<String> identifiersUsed = null;
    String jsFileContent = ProjectsUtil.fetchJsFileContent(project);
    if (jsFileContent.startsWith(HardwareUtil.IDENTIFIERS_USED_PREFIX)) {
      int eol = jsFileContent.indexOf("\n");
      identifiersUsed = new HashSet<>(Arrays.asList(
          jsFileContent.substring(HardwareUtil.IDENTIFIERS_USED_PREFIX.length(), eol).split(",")));
      jsFileContent = jsFileContent.substring(eol);
    }

    addJavascriptInterfaces(hardwareItemMap, identifiersUsed);

    String jsContent = HardwareUtil.upgradeJs(jsFileContent, hardwareItemMap);

    StringBuilder html = new StringBuilder()
        .append("<html><body onload='callRunOpMode()'><script type='text/javascript'>\n");
    FileUtil.readAsset(html, activity.getAssets(), "blocks/runtime.js");
    html.append("\n")
        .append(jsContent)
        .append("\n</script></body></html>\n");
    webView.loadDataWithBaseURL(
        null /* baseUrl */, html.toString(), "text/html", "UTF-8", null /* historyUrl */);
  }

  private void clearScript() {
    removeJavascriptInterfaces();
    if (!javascriptInterfaces.isEmpty()) {
      RobotLog.w(getLogPrefix() + "clearScript - Warning: javascriptInterfaces is not empty.");
    }
    javascriptInterfaces.clear();

    webView.loadDataWithBaseURL(
        null /* baseUrl */, "", "text/html", "UTF-8", null /* historyUrl */);
    nameOfOpModeLoadedIntoWebView.set(null);
  }

  /**
   * Sets the {@link WebView} so that all BlocksOpModes can access it.
   */
  @SuppressLint("setJavaScriptEnabled")
  public static void setActivityAndWebView(Activity a, WebView wv) {
    if (activity == null && webView == null) {
      addOpModeRegistrar();
    }

    activity = a;
    webView = wv;
    webView.getSettings().setJavaScriptEnabled(true);

    webView.setWebChromeClient(new WebChromeClient() {
      @Override
      public boolean onConsoleMessage(ConsoleMessage consoleMessage) {
        return false; // continue with console logging.
      }
    });
  }

  private static void addOpModeRegistrar() {
    RegisteredOpModes.getInstance().addInstanceOpModeRegistrar(new InstanceOpModeRegistrar() {
      @Override public void register(InstanceOpModeManager manager) {
        try {
          // fetchEnabledProjectsWithJavaScript is thread-safe wrt concurrent saves from the browswer
          List<OpModeMeta> projects = ProjectsUtil.fetchEnabledProjectsWithJavaScript();
          for (OpModeMeta opModeMeta : projects) {
            manager.register(opModeMeta, new BlocksOpMode(opModeMeta.name));
          }
        } catch (Exception e) {
          RobotLog.logStackTrace(e);
        }
      }
    });
  }

  /**
   * @deprecated functionality now automatically called by the system
   */
  @Deprecated
  public static void registerAll(OpModeManager manager) {
    RobotLog.w(BlocksOpMode.class.getSimpleName(), "registerAll(OpModeManager) is deprecated and will be removed soon, as calling it is unnecessary in this and future API version");
  }
}
