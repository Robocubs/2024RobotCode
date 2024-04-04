import { WebbitConfig } from '@webbitjs/webbit';
import { LitElement, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { globalStylesCss } from '../styles/styles';

export const incrementableNumberDashboardConfig: Partial<WebbitConfig> = {
  dashboard: {
    displayName: 'Incrementable Number',
  },
  properties: {
    value: { type: 'Number', primary: true, changeEvent: 'change' },
    label: { type: 'String', defaultValue: 'Number' },
    min: { type: 'Number', defaultValue: -1 },
    max: { type: 'Number', defaultValue: 1 },
    step: { type: 'Number', defaultValue: 0.1 },
  },
};

@customElement('team1701-incrementable-number')
export class IncrementableNumber extends LitElement {
  @property({ type: String }) label = 'Number';
  @property({ type: Number }) value = 0;
  @property({ type: Number }) min = -1;
  @property({ type: Number }) max = 1;
  @property({ type: Number }) step = 0.1;

  increment() {
    const value = Math.round((this.value + this.step + Number.EPSILON) / this.step) * this.step;
    this.value = value > this.max ? this.max : value;
    this.#dispatchChange();
  }

  decrement() {
    const value = Math.round((this.value - this.step + Number.EPSILON) / this.step) * this.step;
    this.value = value < this.min ? this.min : value;
    this.#dispatchChange();
  }

  #dispatchChange(): void {
    this.dispatchEvent(
      new CustomEvent('change', {
        detail: { value: this.value },
        bubbles: true,
        composed: true,
      })
    );
  }

  render() {
    return html`
      <div class="flex flex-col gap-1 justify-center text-white font-sans">
        <div class="text-center">${this.label}</div>
        <div class="flex flex-row gap-2 items-center justify-between">
          <button class="p-1 w-6 bg-white rounded text-black font-medium font" @click=${this.decrement}>-</button>
          <div>${this.value.toFixed(2)}</div>
          <button class="p-1 w-6 bg-white rounded text-black font-medium" @click=${this.increment}>+</button>
        </div>
      </div>
    `;
  }

  static styles = [globalStylesCss];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-incrementable-number': IncrementableNumber;
  }
}
