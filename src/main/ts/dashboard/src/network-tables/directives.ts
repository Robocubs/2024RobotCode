import { Store } from '@webbitjs/store';
import { AsyncDirective } from 'lit/async-directive.js';
import { Directive, directive } from 'lit/directive.js';
import { Result } from 'postcss';

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

export class SourceDoubleArray extends AsyncDirective {
  static getSourceValue(store: Store, provider: string, key: string, defaultValue: number[]): number[] {
    const source = store.getSource(provider, key);
    if (!source?.hasValue()) {
      return defaultValue;
    }
    //return source?.hasValue() ? new Uint8Array(source.getValue() as Uint8Array) : defaultValue;
    const view = new DataView(new Uint8Array(source.getValue() as Uint8Array).buffer);
    const doubles = [];
    for (let i = 0; i < view.byteLength / 8; i++) {
      doubles.push(view.getFloat64(i * 8, true));
    }

    return doubles;
  }

  render(store: Store, provider: string, key: string, defaultValue: number[]) {
    store.subscribe(
      provider,
      key,
      () => {
        this.setValue(SourceDoubleArray.getSourceValue(store, provider, key, defaultValue));
      },
      false
    );
    return SourceDoubleArray.getSourceValue(store, provider, key, defaultValue);
  }
}
export const sourceDoubleArray = directive(SourceDoubleArray);
export const ntDoubleArrayDirective = (store: Store) => (key: string, defaultValue: number[]) =>
  sourceDoubleArray(store, 'NetworkTables', key, defaultValue);
