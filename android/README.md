# OpenNeato Android app

Native app (Kotlin, Jetpack Compose, Material 3 Expressive) for the OpenNeato bridge.
It talks to the same HTTP API as the web UI; the client is generated at build time
from [`frontend/api/openapi.yaml`](../frontend/api/openapi.yaml), nothing generated is checked in.

## Build

Requires JDK 21 and the Android SDK (platform 37). From this folder:

```bash
./gradlew :app:testDebugUnitTest :app:assembleDebug   # tests + debug APK
./gradlew :app:assembleRelease                        # release APK (unsigned without a keystore)
```

Release signing reads `ANDROID_KEYSTORE_PATH`, `ANDROID_KEYSTORE_PASSWORD`,
`ANDROID_KEY_ALIAS` and `ANDROID_KEY_PASSWORD` (or the `openneato.*` Gradle properties).

## Releases

`.github/workflows/android.yml` is called by the release and prerelease workflows and
attaches `OpenNeato-android-<version>.apk` to the GitHub release. The keystore lives in
the repository secrets (`ANDROID_KEYSTORE_BASE64` and friends). Every release must be
signed with the same key, otherwise installed apps cannot be updated.

## Trying it without a robot

Run the web mock (`npm run dev` in `frontend/`) and enter `10.0.2.2:5173` as the address
in the emulator.
