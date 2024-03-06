import { dashboardElementConfigs } from '@frc-web-components/fwc/components';
import { Nt4Provider } from '@frc-web-components/fwc/source-providers';
import { createContext } from '@lit/context';
import { Store } from '@webbitjs/store';
import { WebbitConnector } from '@webbitjs/webbit';
import { DirectiveResult } from 'lit/directive.js';
import { componentElementConfigs } from '../components/component-configs';
import { SourceAllianceColor, SourceValue, ntAllianceColorDirective, ntValueDirective } from './directives';

export class NetworkTables {
  private readonly store: Store;
  private readonly provider: Nt4Provider;
  private readonly nt4ValueDirective;
  private readonly nt4AllianceColorDirective;

  constructor(address: string) {
    this.provider = new Nt4Provider();
    this.store = new Store();
    this.store.addSourceProvider('NetworkTables', this.provider);
    this.store.setDefaultSourceProvider('NetworkTables');
    this.provider.connect(address);

    this.nt4ValueDirective = ntValueDirective(this.store);
    this.nt4AllianceColorDirective = ntAllianceColorDirective(this.store);
  }

  bindConnection(rootElement: HTMLElement) {
    new WebbitConnector(rootElement, this.store).addElementConfigs({ ...dashboardElementConfigs, ...componentElementConfigs }, 'FRC');
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

  $value(key: string, defaultValue: unknown): DirectiveResult<typeof SourceValue> {
    return this.nt4ValueDirective(key, defaultValue);
  }

  $allianceColor(): DirectiveResult<typeof SourceAllianceColor> {
    return this.nt4AllianceColorDirective();
  }
}

export const nt4Context = createContext<NetworkTables>('nt4');
