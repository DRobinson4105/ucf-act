/**
 * Expo config plugin that overwrites the default expo-live-activity widget
 * Swift files with our custom ACT ride-tracking UI.
 *
 * Must be listed AFTER "expo-live-activity" in app.json plugins.
 */
const { withDangerousMod } = require("@expo/config-plugins");
const fs = require("fs");
const path = require("path");

const WIDGET_DIR = "LiveActivity";

// ── Custom LiveActivityWidget.swift ─────────────────────────────────────────
const WIDGET_SWIFT = `import ActivityKit
import SwiftUI
import WidgetKit

struct LiveActivityAttributes: ActivityAttributes {
  struct ContentState: Codable, Hashable {
    var title: String
    var subtitle: String?
    var timerEndDateInMilliseconds: Double?
    var progress: Double?
    var imageName: String?
    var dynamicIslandImageName: String?
  }

  var name: String
  var backgroundColor: String?
  var titleColor: String?
  var subtitleColor: String?
  var progressViewTint: String?
  var progressViewLabelColor: String?
  var deepLinkUrl: String?
  var timerType: DynamicIslandTimerType?
  var padding: Int?
  var paddingDetails: PaddingDetails?
  var imagePosition: String?
  var imageWidth: Int?
  var imageHeight: Int?
  var imageWidthPercent: Double?
  var imageHeightPercent: Double?
  var imageAlign: String?
  var contentFit: String?

  enum DynamicIslandTimerType: String, Codable {
    case circular
    case digital
  }

  struct PaddingDetails: Codable, Hashable {
    var top: Int?
    var bottom: Int?
    var left: Int?
    var right: Int?
    var vertical: Int?
    var horizontal: Int?
  }
}

// MARK: - Widget

struct LiveActivityWidget: Widget {
  private static let teal = Color(hex: "#2DD4BF")
  private static let dim = Color.white.opacity(0.5)

  var body: some WidgetConfiguration {
    ActivityConfiguration(for: LiveActivityAttributes.self) { context in
      RideBannerView(state: context.state)
        .activityBackgroundTint(Color(hex: "#111111"))
        .activitySystemActionForegroundColor(.white)
        .applyWidgetURL(from: context.attributes.deepLinkUrl)
    } dynamicIsland: { context in
      let s = context.state

      DynamicIsland {
        DynamicIslandExpandedRegion(.leading) {
          VStack(alignment: .leading, spacing: 2) {
            Text(s.title)
              .font(.subheadline)
              .fontWeight(.semibold)
              .foregroundStyle(.white)
            if let dest = Self.destination(from: s.subtitle) {
              Text(dest)
                .font(.caption)
                .foregroundStyle(Self.dim)
                .lineLimit(1)
            }
          }
          .padding(.leading, 4)
        }

        DynamicIslandExpandedRegion(.trailing) {
          if let dist = Self.distance(from: s.subtitle) {
            Text(dist)
              .font(.caption)
              .fontWeight(.semibold)
              .foregroundStyle(Self.teal)
              .monospacedDigit()
              .padding(.trailing, 4)
          }
        }

        DynamicIslandExpandedRegion(.bottom) {
          RideProgressBar(progress: s.progress ?? 0, tint: Self.teal)
            .padding(.horizontal, 4)
            .padding(.top, 2)
        }

      } compactLeading: {
        Image(systemName: "car.fill")
          .foregroundStyle(Self.teal)
          .font(.system(size: 13, weight: .medium))
          .widgetAccentable()

      } compactTrailing: {
        if let dist = Self.distance(from: s.subtitle) {
          Text(dist)
            .font(.system(size: 12, weight: .semibold, design: .rounded))
            .foregroundStyle(Self.teal)
            .monospacedDigit()
            .contentTransition(.numericText())
        } else {
          Circle()
            .fill(Self.teal)
            .frame(width: 6, height: 6)
        }

      } minimal: {
        Image(systemName: "car.fill")
          .foregroundStyle(Self.teal)
          .font(.system(size: 11))
      }
    }
  }

  static func distance(from subtitle: String?) -> String? {
    guard let subtitle = subtitle else { return nil }
    if let range = subtitle.range(of: " \\u{00B7} ") {
      let d = String(subtitle[subtitle.startIndex..<range.lowerBound])
      return d.isEmpty ? nil : d
    }
    return subtitle
  }

  static func destination(from subtitle: String?) -> String? {
    guard let subtitle = subtitle else { return nil }
    if let range = subtitle.range(of: " \\u{00B7} ") {
      let d = String(subtitle[range.upperBound...])
      return d.isEmpty ? nil : d
    }
    return nil
  }
}

// MARK: - Progress bar with sliding cart

struct RideProgressBar: View {
  let progress: Double
  let tint: Color

  private let trackHeight: CGFloat = 5
  private let dotSize: CGFloat = 6

  var body: some View {
    let clamped = min(max(progress, 0), 1)

    GeometryReader { geo in
      let w = geo.size.width
      let usable = w - dotSize

      ZStack(alignment: .leading) {
        Capsule()
          .fill(Color.white.opacity(0.12))
          .frame(height: trackHeight)
          .padding(.horizontal, dotSize / 2)

        Capsule()
          .fill(
            LinearGradient(
              colors: [tint.opacity(0.6), tint],
              startPoint: .leading,
              endPoint: .trailing
            )
          )
          .frame(width: max(usable * clamped + dotSize, dotSize), height: trackHeight)

        Circle()
          .fill(Color.white.opacity(0.35))
          .frame(width: dotSize, height: dotSize)

        Circle()
          .fill(tint.opacity(0.5))
          .frame(width: dotSize, height: dotSize)
          .frame(maxWidth: .infinity, alignment: .trailing)

        Image(systemName: "car.fill")
          .font(.system(size: 10, weight: .bold))
          .foregroundStyle(tint)
          .offset(x: usable * clamped - 1, y: -11)
      }
    }
    .frame(height: 24)
  }
}

// MARK: - Lock screen banner

struct RideBannerView: View {
  let state: LiveActivityAttributes.ContentState

  private let teal = Color(hex: "#2DD4BF")
  private let dim = Color.white.opacity(0.5)

  var body: some View {
    VStack(alignment: .leading, spacing: 14) {
      HStack(spacing: 12) {
        ZStack {
          RoundedRectangle(cornerRadius: 10, style: .continuous)
            .fill(teal.opacity(0.15))
            .frame(width: 38, height: 38)
          Image(systemName: "car.fill")
            .font(.system(size: 16, weight: .semibold))
            .foregroundStyle(teal)
        }

        VStack(alignment: .leading, spacing: 2) {
          Text(state.title)
            .font(.subheadline)
            .fontWeight(.semibold)
            .foregroundStyle(.white)

          if let dest = LiveActivityWidget.destination(from: state.subtitle) {
            Text(dest)
              .font(.caption)
              .foregroundStyle(dim)
              .lineLimit(1)
          }
        }

        Spacer()

        if let dist = LiveActivityWidget.distance(from: state.subtitle) {
          Text(dist)
            .font(.subheadline)
            .fontWeight(.bold)
            .foregroundStyle(teal)
            .monospacedDigit()
            .contentTransition(.numericText())
        }
      }

      RideProgressBar(progress: state.progress ?? 0, tint: teal)
    }
    .padding(.horizontal, 20)
    .padding(.vertical, 16)
  }
}
`;

// ── Custom LiveActivityView.swift (minimal — banner lives in Widget file) ───
const VIEW_SWIFT = `import SwiftUI
import WidgetKit

// Lock screen banner is rendered by RideBannerView inside LiveActivityWidget.swift.
// This file is kept so the Xcode target file list stays stable.
`;

function withCustomLiveActivity(config) {
  return withDangerousMod(config, [
    "ios",
    (config) => {
      const projectRoot = config.modRequest.projectRoot;
      const widgetPath = path.join(projectRoot, "ios", WIDGET_DIR);

      if (fs.existsSync(widgetPath)) {
        fs.writeFileSync(
          path.join(widgetPath, "LiveActivityWidget.swift"),
          WIDGET_SWIFT
        );
        fs.writeFileSync(
          path.join(widgetPath, "LiveActivityView.swift"),
          VIEW_SWIFT
        );
        console.log("[withCustomLiveActivity] Widget files patched");
      }

      return config;
    },
  ]);
}

module.exports = withCustomLiveActivity;
