import { FrcDashboard, dashboardElementConfigs } from '@frc-web-components/fwc';
import { Nt4Provider } from '@frc-web-components/fwc/source-providers';
import { createContext } from '@lit/context';
import { Store } from '@webbitjs/store';
import { DirectiveResult } from 'lit/directive.js';
import { SourcePose2d, SourceValue, ntPose2dDirective, ntValueDirective } from './directives';

export class NetworkTables {
  private readonly store: Store;
  private readonly provider: Nt4Provider;
  private readonly nt4ValueDirective;
  private readonly nt4Pose2dDirective;

  constructor(address: string) {
    const dashboard = new FrcDashboard(document.body);
    const provider = new Nt4Provider();
    dashboard.addSourceProvider('NetworkTables', provider);
    dashboard.setDefaultSourceProvider('NetworkTables');
    dashboard.addElements(dashboardElementConfigs, 'FRC');
    provider.connect(address);

    this.store = dashboard.getStore();
    this.provider = provider;
    this.nt4ValueDirective = ntValueDirective(this.store);
    this.nt4Pose2dDirective = ntPose2dDirective(this.store);
  }

  getValue<T>(key: string, defaultValue: T): T {
    if (!this.store.getSource('NetworkTables', key)?.hasValue()) {
      return defaultValue;
    }
    return this.store.getSourceValue('NetworkTables', key) as T;
  }

  setValue(key: string, value: unknown) {
    this.provider.userUpdate(key, value);
  }

  addKeyListener<T>(key: string, callback: (key: string, value: T) => unknown, immediateNotify: boolean) {
    return this.store.subscribe(
      'NetworkTables',
      key,
      (value: unknown) => {
        callback(key, value as T);
      },
      immediateNotify
    );
  }

  addGlobalListener(callback: (key: string, value: unknown) => unknown, immediateNotify: boolean) {
    return this.store.subscribeAll(
      'NetworkTables',
      (value: unknown, key: string) => {
        callback(key, value);
      },
      immediateNotify
    );
  }

  getStore() {
    return this.store;
  }

  $value(key: string, value: unknown): DirectiveResult<typeof SourceValue> {
    return this.nt4ValueDirective(key, value);
  }

  $pose2d(key: string, value: number[]): DirectiveResult<typeof SourcePose2d> {
    return this.nt4Pose2dDirective(key, value);
  }
}

export default NetworkTables;

export const nt4Context = createContext<NetworkTables>('nt4');
