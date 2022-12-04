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
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/**
 * A class that provides JavaScript access to {@link Quaternion}.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
class QuaternionAccess extends Access {

  QuaternionAccess(BlocksOpMode blocksOpMode, String identifier) {
    super(blocksOpMode, identifier, "Quaternion");
  }

  private Quaternion checkQuaternion(Object quaternionArg) {
    return checkArg(quaternionArg, Quaternion.class, "quaternion");
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getW(Object quaternionArg) {
    try {
      startBlockExecution(BlockType.GETTER, ".W");
      Quaternion quaternion = checkQuaternion(quaternionArg);
      if (quaternion != null) {
        return quaternion.w;
      }
      return 0;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getX(Object quaternionArg) {
    try {
      startBlockExecution(BlockType.GETTER, ".X");
      Quaternion quaternion = checkQuaternion(quaternionArg);
      if (quaternion != null) {
        return quaternion.x;
      }
      return 0;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getY(Object quaternionArg) {
    try {
      startBlockExecution(BlockType.GETTER, ".Y");
      Quaternion quaternion = checkQuaternion(quaternionArg);
      if (quaternion != null) {
        return quaternion.y;
      }
      return 0;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getZ(Object quaternionArg) {
    try {
      startBlockExecution(BlockType.GETTER, ".Z");
      Quaternion quaternion = checkQuaternion(quaternionArg);
      if (quaternion != null) {
        return quaternion.z;
      }
      return 0;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public long getAcquisitionTime(Object quaternionArg) {
    try {
      startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
      Quaternion quaternion = checkQuaternion(quaternionArg);
      if (quaternion != null) {
        return quaternion.acquisitionTime;
      }
      return 0;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getMagnitude(Object quaternionArg) {
    try {
      startBlockExecution(BlockType.GETTER, ".Magnitude");
      Quaternion quaternion = checkQuaternion(quaternionArg);
      if (quaternion != null) {
        return quaternion.magnitude();
      }
      return 0;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Quaternion create() {
    try {
      startBlockExecution(BlockType.CREATE, "");
      return new Quaternion();
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Quaternion create_withArgs(float w, float x, float y, float z, long acquisitionTime) {
    try {
      startBlockExecution(BlockType.CREATE, "");
      return new Quaternion(w, x, y, z, acquisitionTime);
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Quaternion normalized(Object quaternionArg) {
    try {
      startBlockExecution(BlockType.FUNCTION, ".normalized");
      Quaternion quaternion = checkQuaternion(quaternionArg);
      if (quaternion != null) {
        return quaternion.normalized();
      }
      return null;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Quaternion congugate(Object quaternionArg) {
    try {
      startBlockExecution(BlockType.FUNCTION, ".congugate");
      Quaternion quaternion = checkQuaternion(quaternionArg);
      if (quaternion != null) {
        return quaternion.congugate();
      }
      return null;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }
}
