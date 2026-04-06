import ActivityKit
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

// MARK: - State detection helpers

private enum RidePhase {
  case searching   // "Finding your cart…"
  case enRoute     // "Cart on the way"
  case arrived     // "Cart arrived!" — user needs to board
  case riding      // "On your way"
  case ended       // "Ride complete"

  init(title: String) {
    let t = title.lowercased()
    if t.contains("finding") { self = .searching }
    else if t.contains("arrived") { self = .arrived }
    else if t.contains("on your way") { self = .riding }
    else if t.contains("complete") || t.contains("cancelled") { self = .ended }
    else { self = .enRoute }
  }
}

// MARK: - Widget

struct LiveActivityWidget: Widget {
  static let teal = Color(hex: "#2DD4BF")
  static let dim = Color.white.opacity(0.5)

  var body: some WidgetConfiguration {
    ActivityConfiguration(for: LiveActivityAttributes.self) { context in
      RideBannerView(state: context.state)
        .activityBackgroundTint(Color(hex: "#111111"))
        .activitySystemActionForegroundColor(.white)
        .applyWidgetURL(from: context.attributes.deepLinkUrl)
    } dynamicIsland: { context in
      let s = context.state
      let phase = RidePhase(title: s.title)

      DynamicIsland {
        // ── Expanded ─────────────────────────────────
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
          if phase == .arrived {
            // Boarding prompt
            Text("Tap to board")
              .font(.caption2)
              .fontWeight(.bold)
              .foregroundStyle(.black)
              .padding(.horizontal, 8)
              .padding(.vertical, 4)
              .background(Self.teal, in: Capsule())
              .padding(.trailing, 4)
          } else if let dist = Self.distance(from: s.subtitle) {
            Text(dist)
              .font(.caption)
              .fontWeight(.semibold)
              .foregroundStyle(Self.teal)
              .monospacedDigit()
              .padding(.trailing, 4)
          }
        }

        DynamicIslandExpandedRegion(.bottom) {
          if phase == .searching {
            // Indeterminate searching indicator
            SearchingBar(tint: Self.teal)
              .padding(.horizontal, 4)
              .padding(.top, 2)
          } else if phase == .arrived {
            // Full progress — cart is here
            RideProgressBar(progress: 1.0, tint: Self.teal)
              .padding(.horizontal, 4)
              .padding(.top, 2)
          } else {
            RideProgressBar(progress: s.progress ?? 0, tint: Self.teal)
              .padding(.horizontal, 4)
              .padding(.top, 2)
          }
        }

      } compactLeading: {
        Image(systemName: phase == .arrived ? "figure.wave" : "car.fill")
          .foregroundStyle(Self.teal)
          .font(.system(size: 13, weight: .medium))
          .widgetAccentable()

      } compactTrailing: {
        if phase == .searching {
          // Pulsing dot while searching
          Circle()
            .fill(Self.teal)
            .frame(width: 6, height: 6)
        } else if phase == .arrived {
          Text("Board")
            .font(.system(size: 11, weight: .bold, design: .rounded))
            .foregroundStyle(Self.teal)
        } else if let dist = Self.distance(from: s.subtitle) {
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
        Image(systemName: phase == .arrived ? "figure.wave" : "car.fill")
          .foregroundStyle(Self.teal)
          .font(.system(size: 11))
      }
    }
  }

  // Parses "0.3 mi · Engineering I" → "0.3 mi"
  static func distance(from subtitle: String?) -> String? {
    guard let subtitle = subtitle else { return nil }
    if let range = subtitle.range(of: " \u{00B7} ") {
      let d = String(subtitle[subtitle.startIndex..<range.lowerBound])
      return d.isEmpty ? nil : d
    }
    // If no separator, subtitle is just a name — no distance
    if subtitle.contains("mi") || subtitle.contains("ft") {
      return subtitle
    }
    return nil
  }

  // Parses "0.3 mi · Engineering I" → "Engineering I"
  static func destination(from subtitle: String?) -> String? {
    guard let subtitle = subtitle else { return nil }
    if let range = subtitle.range(of: " \u{00B7} ") {
      let d = String(subtitle[range.upperBound...])
      return d.isEmpty ? nil : d
    }
    // If no separator, subtitle is just a destination name
    if !subtitle.contains("mi") && !subtitle.contains("ft") {
      return subtitle
    }
    return nil
  }
}

// MARK: - Searching indicator (indeterminate)

struct SearchingBar: View {
  let tint: Color

  var body: some View {
    GeometryReader { geo in
      let w = geo.size.width

      ZStack(alignment: .leading) {
        Capsule()
          .fill(Color.white.opacity(0.12))
          .frame(height: 5)

        Capsule()
          .fill(
            LinearGradient(
              colors: [tint.opacity(0.3), tint, tint.opacity(0.3)],
              startPoint: .leading,
              endPoint: .trailing
            )
          )
          .frame(width: w * 0.35, height: 5)
          .offset(x: w * 0.32)
      }
    }
    .frame(height: 24)
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
    let phase = RidePhase(title: state.title)

    VStack(alignment: .leading, spacing: 14) {
      // Header row
      HStack(spacing: 12) {
        ZStack {
          RoundedRectangle(cornerRadius: 10, style: .continuous)
            .fill(phase == .arrived ? teal.opacity(0.25) : teal.opacity(0.15))
            .frame(width: 38, height: 38)
          Image(systemName: phase == .arrived ? "figure.wave" : "car.fill")
            .font(.system(size: 16, weight: .semibold))
            .foregroundStyle(teal)
        }

        VStack(alignment: .leading, spacing: 2) {
          Text(state.title)
            .font(.subheadline)
            .fontWeight(.semibold)
            .foregroundStyle(.white)

          if phase == .arrived {
            Text("Tap to open app & confirm boarding")
              .font(.caption2)
              .foregroundStyle(teal)
          } else if let dest = LiveActivityWidget.destination(from: state.subtitle) {
            Text(dest)
              .font(.caption)
              .foregroundStyle(dim)
              .lineLimit(1)
          }
        }

        Spacer()

        if phase == .arrived {
          Text("Board")
            .font(.caption)
            .fontWeight(.bold)
            .foregroundStyle(.black)
            .padding(.horizontal, 12)
            .padding(.vertical, 6)
            .background(teal, in: Capsule())
        } else if let dist = LiveActivityWidget.distance(from: state.subtitle) {
          Text(dist)
            .font(.subheadline)
            .fontWeight(.bold)
            .foregroundStyle(teal)
            .monospacedDigit()
            .contentTransition(.numericText())
        }
      }

      // Progress / searching bar
      if phase == .searching {
        SearchingBar(tint: teal)
      } else if phase == .arrived {
        RideProgressBar(progress: 1.0, tint: teal)
      } else {
        RideProgressBar(progress: state.progress ?? 0, tint: teal)
      }
    }
    .padding(.horizontal, 20)
    .padding(.vertical, 16)
  }
}
