/*
 * Copyright (c) 2016 Craig MacFarlane
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Craig MacFarlane nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcore.internal.opmode;

import android.content.Context;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.os.Build;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import dalvik.system.DexFile;
@SuppressWarnings("WeakerAccess")
public class InstantRunHelper {

    public static final String TAG = "InstantRunHelper";

    /**
     * Find all of the classes in secondary apk files created for Instant Run.
     * This does NOT find classes in the base APK file.
     *
     * @param context the application context
     * @return A list of class names
     */
    public static List<String> getAllClassNames(Context context)
    {
        ApplicationInfo applicationInfo = null;
        List<String> classNames = new ArrayList<String>();

        try {
            applicationInfo = context.getPackageManager().getApplicationInfo(context.getPackageName(), 0);
        } catch (PackageManager.NameNotFoundException e) {
            RobotLog.ee(TAG, e, "Could not obtain application info for class scanning");
            return classNames;
        }

        // For the current iteration of Instant Run, and we need access to applicationInfo.splitSourceDirs
        String[] apkFiles = applicationInfo.splitSourceDirs;
        if (apkFiles != null) {
            for (String path : apkFiles) {
                try {
                    DexFile dexFile = new DexFile(path);
                    try {
                        classNames.addAll(Collections.list(dexFile.entries()));
                    } finally {
                        dexFile.close();
                    }
                } catch (IOException e) {
                    RobotLog.ee(TAG, e,"Error accessing apk file: " + path + ", IOException: " + e.toString());
                }
            }
        }
        return classNames;
    }
}
