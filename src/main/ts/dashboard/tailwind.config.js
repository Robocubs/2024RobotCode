/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,ts,html,css}'],
  theme: {
    extend: {
      colors: {
        primary: 'rgb(156,32,49)',
      },
    },
  },
  plugins: [],
};
