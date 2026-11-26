#pragma once

#include <android_native_app_glue.h>
#include <jni.h>

#include <string>

#include "logger.h"

inline void announce_recording_event(android_app* app, const std::string& event) {
  if (!app || !app->activity || !app->activity->vm || !app->activity->clazz)
    return;

  JavaVM* vm = app->activity->vm;
  JNIEnv* env = nullptr;
  bool attached = false;
  const jint status = vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
  if (status == JNI_EDETACHED) {
    if (vm->AttachCurrentThread(&env, nullptr) != JNI_OK)
      return;
    attached = true;
  } else if (status != JNI_OK) {
    return;
  }

  jclass activity_class = env->GetObjectClass(app->activity->clazz);
  jmethodID method = activity_class
                         ? env->GetMethodID(activity_class, "announceRecordingEvent",
                                            "(Ljava/lang/String;)V")
                         : nullptr;
  if (method) {
    jstring value = env->NewStringUTF(event.c_str());
    if (value) {
      env->CallVoidMethod(app->activity->clazz, method, value);
      env->DeleteLocalRef(value);
    }
  }
  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    LOGE("Could not deliver recording voice cue to Android activity");
  }
  if (activity_class)
    env->DeleteLocalRef(activity_class);
  if (attached)
    vm->DetachCurrentThread();
}
