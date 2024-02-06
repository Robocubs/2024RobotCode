import { Store } from '@webbitjs/store';
import { AsyncDirective } from 'lit/async-directive.js';
import { directive } from 'lit/directive.js';

export class SourceValue extends AsyncDirective {
  static getSourceValue(store: Store, provider: string, key: string, defaultValue: unknown): unknown {
    const source = store.getSource(provider, key);
    const value = source?.hasValue() ? source.getValue() : defaultValue;
    return typeof value === 'string' ? value : JSON.stringify(value);
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

export class SourcePose2d extends AsyncDirective {
  static getSourceValue(store: Store, provider: string, key: string, defaultValue: number[]): unknown {
    const source = store.getSource(provider, key);
    if (!source?.hasValue()) {
      return defaultValue;
    }

    const view = new DataView(new Uint8Array(source.getValue() as Uint8Array).buffer);
    const pose = [];
    for (let i = 0; i < 3; i++) {
      pose.push(view.getFloat64(i * 8, true));
    }

    return pose;
  }

  render(store: Store, provider: string, key: string, defaultValue: number[]) {
    store.subscribe(
      provider,
      key,
      () => {
        this.setValue(SourcePose2d.getSourceValue(store, provider, key, defaultValue));
      },
      false
    );
    return SourcePose2d.getSourceValue(store, provider, key, defaultValue);
  }
}

export const sourcePose2d = directive(SourcePose2d);

export const ntPose2dDirective = (store: Store) => (key: string, defaultValue: number[]) => sourcePose2d(store, 'NetworkTables', key, defaultValue);