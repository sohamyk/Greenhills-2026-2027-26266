plugins {
    id("dev.frozenmilk.teamcode") version "12.0.0-1.2.1"
    id("dev.frozenmilk.sinister.sloth.load") version "0.3.1"
}

repositories {
    mavenCentral()
    maven("https://mymaven.bylazar.com/releases")
}

ftc {
    kotlin()

    sdk.TeamCode()
    implementation(dairy.Sloth)
}

dependencies {
    implementation("com.pedropathing:revhub:3.0.2")
    implementation("com.pedropathing:tuning:1.0.0")
    implementation("dev.nextftc.v2:control:0.3.0")
    implementation("dev.nextftc.v2:hardware:0.3.0")
    implementation("dev.nextftc.v2:robot:0.3.0")
}
