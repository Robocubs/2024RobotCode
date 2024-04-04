import { WebbitConfig } from '@webbitjs/webbit';
import { alertsDashboardConfig } from './alerts';
import { commandsDashboardConfig } from './commands';
import { incrementableNumberDashboardConfig } from './incrementable-number';

export const componentElementConfigs = {
  'team1701-alerts': alertsDashboardConfig,
  'team1701-commands': commandsDashboardConfig,
  'team1701-incrementable-number': incrementableNumberDashboardConfig,
} as Record<string, WebbitConfig>;
