/**
 * Expo config plugin that overwrites the default expo-live-activity widget
 * Swift files with our custom ACT ride-tracking UI.
 *
 * Must be listed AFTER "expo-live-activity" in app.json plugins.
 *
 * Source files live in plugins/live-activity-swift/ and are copied to
 * ios/LiveActivity/ during prebuild.
 */
const { withDangerousMod } = require("@expo/config-plugins");
const fs = require("fs");
const path = require("path");

const WIDGET_DIR = "LiveActivity";
const SWIFT_SRC = path.join(__dirname, "live-activity-swift");

function withCustomLiveActivity(config) {
  return withDangerousMod(config, [
    "ios",
    (config) => {
      const projectRoot = config.modRequest.projectRoot;
      const widgetPath = path.join(projectRoot, "ios", WIDGET_DIR);

      if (fs.existsSync(widgetPath) && fs.existsSync(SWIFT_SRC)) {
        for (const file of fs.readdirSync(SWIFT_SRC)) {
          if (file.endsWith(".swift")) {
            fs.copyFileSync(
              path.join(SWIFT_SRC, file),
              path.join(widgetPath, file)
            );
          }
        }
        console.log("[withCustomLiveActivity] Widget files patched");
      }

      return config;
    },
  ]);
}

module.exports = withCustomLiveActivity;
