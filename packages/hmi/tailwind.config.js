/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        background: '#111111',
        surface: '#1A1A1A',
        surfaceLight: '#222222',
        border: '#2A2A2A',
        primary: '#2DD4BF',
        secondary: '#1A1A1A',
        accent: '#2DD4BF',
        success: '#10B981',
        warning: '#F59E0B',
        danger: '#EF4444',
        destination: '#F87171',
        text: {
          primary: '#F5F5F5',
          secondary: '#888888',
          muted: '#52525b'
        }
      },
      fontFamily: {
        sans: ['Inter', 'system-ui', 'sans-serif'],
      },
      borderRadius: {
        'xl': '12px',
        '2xl': '16px',
      }
    },
  },
  plugins: [],
}
