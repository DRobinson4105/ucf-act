/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ["./app/**/*.{js,jsx,ts,tsx}", "./components/**/*.{js,jsx,ts,tsx}"],
  presets: [require("nativewind/preset")],
  theme: {
    extend: {
      colors: {
        primary: "#FFC904",
        secondary: "#1A1A1A",
        accent: "#FFC904",
        background: "#1A1A1A",
        surface: "#2A2A2A",
        card: "#333333",
        text: "#FFFFFF",
        textSecondary: "#9CA3AF",
        border: "#3A3A3A",
        error: "#EF4444",
        success: "#10B981",
        warning: "#F59E0B",
        white: "#FFFFFF",
        black: "#000000",
      },
      fontFamily: {
        sans: ["Inter_400Regular", "System"],
        bold: ["Inter_700Bold", "System"],
      },
    },
  },
  plugins: [],
}
