/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ["./app/**/*.{js,jsx,ts,tsx}", "./components/**/*.{js,jsx,ts,tsx}"],
  presets: [require("nativewind/preset")],
  theme: {
    extend: {
      colors: {
        primary: "#2DD4BF",
        secondary: "#1A1A1A",
        accent: "#2DD4BF",
        background: "#111111",
        surface: "#1A1A1A",
        card: "#222222",
        text: "#F5F5F5",
        textSecondary: "#888888",
        border: "#2A2A2A",
        error: "#EF4444",
        success: "#10B981",
        warning: "#F59E0B",
        destination: "#F87171",
        muted: "#52525b",
        white: "#FFFFFF",
        black: "#111111",
      },
      fontFamily: {
        sans: ["Inter_400Regular", "System"],
        bold: ["Inter_700Bold", "System"],
      },
    },
  },
  plugins: [],
}
