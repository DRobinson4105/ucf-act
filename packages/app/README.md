# ACT — Autonomous Campus Taxi (Mobile App)

Expo (React Native) app for the UCF Autonomous Campus Taxi system.

---

## Prerequisites

| Tool | Version | Install |
|------|---------|---------|
| macOS | 13+ | — |
| Xcode | 15+ | Mac App Store |
| Xcode CLI tools | — | `xcode-select --install` |
| Node.js | 18+ | [nodejs.org](https://nodejs.org) or `brew install node` |
| Bun | 1.0+ | `curl -fsSL https://bun.sh/install \| bash` |
| EAS CLI | any | `npm install -g eas-cli` |

After installing Xcode, open it once so it can finish setup, then accept the license:
install ios packages from xcode

```bash
sudo xcodebuild -license accept
```

Make sure an iOS simulator is installed. Open Xcode → Settings → Platforms → install **iOS 17+**.


---

## 2. Environment variables

Create a `.env.local` file in `packages/app/`. Get the values from the team:

```env
EXPO_PUBLIC_CONVEX_URL=https://<deployment>.convex.cloud

APPLE_ID=com.ucf.act
AUTH_APPLE_KEY_ID=<key-id>
AUTH_APPLE_TEAM_ID=<team-id>
AUTH_APPLE_PRIVATE_KEY='-----BEGIN PRIVATE KEY-----
<private key contents>
-----END PRIVATE KEY-----'
AUTH_SECRET=<hex-secret>
CART_BRIDGE_SECRET=<hex-secret>
```

> The app talks to the shared Convex backend — you do not need to run Convex locally.

---

## 3. Build the dev client (first time only)

The app uses `expo-dev-client`, so it cannot run in Expo Go — it needs a native build. This step compiles the iOS app and installs it on the simulator. You only need to redo this if native dependencies change.

### Local build

Builds directly on your Mac using Xcode. No EAS account needed.

```bash
npx expo run:ios --device
```

You will get a list here, click iphone 17 pro to build on

Xcode will compile, install the app on the simulator, and launch Metro automatically. This takes 5–10 minutes on first run. If doesnt open in simulator press I

---

## Subsequent runs

Once the dev client is installed you only need:

```bash
bun start
```

Then press `i`. You do not need to rebuild unless you add or change a native dependency.

---

## Troubleshooting

**Simulator not found**
Open Xcode → Settings → Platforms and install an iOS 17+ simulator. Changes with every IOS version so reinstall

**`bun: command not found`**
Restart your terminal after installing bun.

**`error: could not find iPhone simulator`**
Run `xcrun simctl list` to see available simulators. If none exist, add one in Xcode → Settings → Platforms.

**App opens but shows a blank screen / Convex errors**
Confirm `.env.local` exists and has `EXPO_PUBLIC_CONVEX_URL` set correctly. The file must be at `packages/app/.env.local`.

**Rebuild required** (after pulling changes that modify `package.json`, `app.json`, or native files):
```bash
bun install
npx expo run:ios --device
```
