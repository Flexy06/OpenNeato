plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.compose)
    alias(libs.plugins.kotlin.serialization)
    alias(libs.plugins.openapi.generator)
}

// API client generated from the same spec the firmware and the web UI use
// (frontend/api/openapi.yaml is the single source of truth). Nothing generated is checked in.
val apiSpec = rootProject.file("../frontend/api/openapi.yaml")
val apiOut = layout.buildDirectory.dir("generated/openapi")

openApiGenerate {
    generatorName.set("kotlin")
    library.set("jvm-okhttp4")
    inputSpec.set(apiSpec.absolutePath)
    outputDir.set(apiOut.get().asFile.absolutePath)
    packageName.set("io.github.flexy06.openneato.api")
    configOptions.set(
        mapOf(
            "serializationLibrary" to "kotlinx_serialization",
            "dateLibrary" to "string",
            "enumPropertyNaming" to "UPPERCASE",
            "omitGradleWrapper" to "true",
            "sourceFolder" to "src/main/kotlin",
        ),
    )
    generateApiTests.set(false)
    generateModelTests.set(false)
    generateApiDocumentation.set(false)
    generateModelDocumentation.set(false)
}

// Release signing from CI secrets (or ~/.gradle/gradle.properties); unsigned otherwise
val keystorePath = providers.environmentVariable("ANDROID_KEYSTORE_PATH").orNull
    ?: providers.gradleProperty("openneato.keystore").orNull

android {
    namespace = "io.github.flexy06.openneato"
    compileSdk = 37

    defaultConfig {
        applicationId = "io.github.flexy06.openneato"
        minSdk = 26
        targetSdk = 36
        versionCode = (providers.gradleProperty("appVersionCode").orNull ?: "1").toInt()
        versionName = providers.gradleProperty("appVersionName").orNull ?: "dev"
    }

    signingConfigs {
        if (keystorePath != null) {
            create("release") {
                storeFile = file(keystorePath)
                storePassword = providers.environmentVariable("ANDROID_KEYSTORE_PASSWORD").orNull
                    ?: providers.gradleProperty("openneato.storePassword").orNull
                keyAlias = providers.environmentVariable("ANDROID_KEY_ALIAS").orNull
                    ?: providers.gradleProperty("openneato.keyAlias").orNull
                keyPassword = providers.environmentVariable("ANDROID_KEY_PASSWORD").orNull
                    ?: providers.gradleProperty("openneato.keyPassword").orNull
            }
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = true
            isShrinkResources = true
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
            if (keystorePath != null) signingConfig = signingConfigs.getByName("release")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    buildFeatures {
        compose = true
        buildConfig = true
    }

}

// Generated client sources (written by openApiGenerate, which runs before every build)
androidComponents {
    onVariants { variant ->
        val dir = apiOut.get().dir("src/main/kotlin").asFile.apply { mkdirs() }
        variant.sources.java?.addStaticSourceDirectory(dir.absolutePath)
    }
}

kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17)
        optIn.add("androidx.compose.material3.ExperimentalMaterial3Api")
        optIn.add("androidx.compose.material3.ExperimentalMaterial3ExpressiveApi")
    }
}

tasks.named("preBuild") { dependsOn("openApiGenerate") }

dependencies {
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.activity.compose)
    implementation(libs.androidx.lifecycle.runtime.compose)
    implementation(libs.androidx.lifecycle.viewmodel.compose)
    implementation(libs.androidx.navigation.compose)
    implementation(libs.androidx.datastore.preferences)
    implementation(platform(libs.compose.bom))
    implementation(libs.compose.ui)
    implementation(libs.compose.ui.tooling.preview)
    implementation(libs.compose.material3)
    implementation(libs.compose.material.icons)
    implementation(libs.okhttp)
    implementation(libs.kotlinx.serialization.json)
    debugImplementation(libs.compose.ui.tooling)
    testImplementation(libs.junit)
}
