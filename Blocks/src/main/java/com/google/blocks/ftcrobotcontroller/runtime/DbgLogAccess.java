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

import android.webkit.JavascriptInterface;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * A class that provides JavaScript access to DbgLog.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
class DbgLogAccess extends Access {

  public static final String TAG = "DbgLog";

  DbgLogAccess(BlocksOpMode blocksOpMode, String identifier) {
    super(blocksOpMode, identifier, "DbgLog");
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void msg(String message) {
    try {
      startBlockExecution(BlockType.FUNCTION, ".msg");
      RobotLog.ii(TAG, message);
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void error(String message) {
    try {
      startBlockExecution(BlockType.FUNCTION, ".error");
      RobotLog.ee(TAG, message);
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }
}
