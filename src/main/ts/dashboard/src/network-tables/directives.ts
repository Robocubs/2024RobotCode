import { Store } from '@webbitjs/store';
import { AsyncDirective } from 'lit/async-directive.js';
import { directive } from 'lit/directive.js';

export class SourceValue extends AsyncDirective {
  static getSourceValue(store: Store, provider: string, key: string, defaultValue: unknown): unknown {
    const source = store.getSource(provider, key);
    return source?.hasValue() ? source.getValue() : defaultValue;
  }

  render(store: Store, provider: string, key: string, defaultValue: unknown) {
    store.subscribe(
      provider,
      key,
      () => {
        this.setValue(SourceValue.getSourceValue(store, provider, key, defaultValue));
      },
      false
    );
    return SourceValue.getSourceValue(store, provider, key, defaultValue);
  }
}
export const sourceValue = directive(SourceValue);
export const ntValueDirective = (store: Store) => (key: string, defaultValue: unknown) => sourceValue(store, 'NetworkTables', key, defaultValue);

export class SourceAllianceColor extends AsyncDirective {
  static getSourceValue(store: Store, provider: string, key: string, defaultValue: string): string {
    const source = store.getSource(provider, key);
    return source?.hasValue() ? (source.getValue() ? 'red' : 'blue') : defaultValue;
  }

  render(store: Store, provider: string, key: string, defaultValue: string) {
    store.subscribe(
      provider,
      key,
      () => {
        this.setValue(SourceAllianceColor.getSourceValue(store, provider, key, defaultValue));
      },
      false
    );
    return SourceAllianceColor.getSourceValue(store, provider, key, defaultValue);
  }
}
export const sourceAllianceColor = directive(SourceAllianceColor);
export const ntAllianceColorDirective = (store: Store) => () => sourceAllianceColor(store, 'NetworkTables', '/FMSInfo/IsRedAlliance', 'blue');
