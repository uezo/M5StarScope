import os

Import("env")

# arduinoWebSockets 2.7.3 includes ESP32's WiFi headers but its PlatformIO
# manifest does not declare those framework libraries as dependencies. Expose
# the two Arduino ESP32 2.x framework include directories it needs.
framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
if framework_dir:
    env.AppendUnique(
        CPPPATH=[
            os.path.join(framework_dir, "libraries", "WiFi", "src"),
            os.path.join(framework_dir, "libraries", "WiFiClientSecure", "src"),
        ]
    )
