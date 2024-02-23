import '@frc-web-components/fwc/components';
import './components';
import { DashboardThemes, darkTheme } from '@frc-web-components/fwc/themes';
import { customDarkTheme } from './styles/themes';
export * from './dashboard';

const themes = new DashboardThemes();
themes.addThemeRules('dark', { ...darkTheme, ...customDarkTheme });
themes.setTheme(document.body, 'dark');
