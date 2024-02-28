import { WebbitConfig } from '@webbitjs/webbit';
import { alertsDashboardConfig } from './alerts';

export const componentElementConfigs = {
  'team1701-alerts': alertsDashboardConfig,
} as Record<string, WebbitConfig>;
