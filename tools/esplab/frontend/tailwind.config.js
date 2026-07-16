/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{ts,tsx}'],
  darkMode: 'class',
  theme: {
    extend: {
      colors: {
        // system-state palette (healthy / recovering / failed / idle)
        ok: '#34d399',
        warn: '#fbbf24',
        bad: '#fb7185',
        idle: '#64748b',
      },
    },
  },
  plugins: [],
}
