import { dashboardElementConfigs } from '@frc-web-components/fwc/components';
import { Nt4Provider } from '@frc-web-components/fwc/source-providers';
import { createContext } from '@lit/context';
import { Store } from '@webbitjs/store';
import { WebbitConfig, WebbitConnector } from '@webbitjs/webbit';
import { Directive, DirectiveResult } from 'lit/directive.js';
import { SourceDoubleArray, SourceValue, ntDoubleArrayDirective, ntValueDirective } from './directives';

export class NetworkTables {
  private readonly store: Store;
  private readonly provider: Nt4Provider;
  private readonly nt4ValueDirective;
  private readonly nt4DoubleArrayDirective;

  constructor(address: string) {
    this.provider = new Nt4Provider();
    this.store = new Store();
    this.store.addSourceProvider('NetworkTables', this.provider);
    this.store.setDefaultSourceProvider('NetworkTables');
    this.provider.connect(address);

    this.nt4ValueDirective = ntValueDirective(this.store);
    this.nt4DoubleArrayDirective = ntDoubleArrayDirective(this.store);
  }

  bindConnection(rootElement: HTMLElement) {
    new WebbitConnector(rootElement, this.store).addElementConfigs(dashboardElementConfigs as Record<string, Partial<WebbitConfig>>, 'FRC');
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

  $doubleArray(key: string, value: number[]): DirectiveResult<typeof SourceDoubleArray> {
    return this.nt4DoubleArrayDirective(key, value);
  }
}

export const nt4Context = createContext<NetworkTables>('nt4');
