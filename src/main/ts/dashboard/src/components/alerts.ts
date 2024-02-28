import { WebbitConfig } from '@webbitjs/webbit';
import { LitElement, css, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { map } from 'lit/directives/map.js';
import { globalStylesCss } from '../styles/styles';

export const alertsDashboardConfig: Partial<WebbitConfig> = {
  dashboard: {
    displayName: 'Alerts',
  },
  properties: {
    errors: { type: 'Array', defaultValue: [] },
    warnings: { type: 'Array', defaultValue: [] },
    infos: { type: 'Array', defaultValue: [] },
  },
};

@customElement('team1701-alerts')
export class Alerts extends LitElement {
  @property({ type: Array }) errors: string[] = [];
  @property({ type: Array }) warnings: string[] = [];
  @property({ type: Array }) infos: string[] = [];

  render() {
    return html`<div class="flex flex-col gap-4 h-full">
      <div class="flex flex-col gap-4 text-white">
        <div>Alerts</div>
        <ul class="list-disc text-red-500 ml-5">
          ${map(this.errors, (e) => html`<li>${e}</li>`)}
        </ul>
        <ul class="list-disc text-yellow-500 ml-5">
          ${map(this.warnings, (e) => html`<li>${e}</li>`)}
        </ul>
        <ul class="list-disc text-blue-500 ml-5">
          ${map(this.infos, (e) => html`<li>${e}</li>`)}
        </ul>
      </div>
    </div>`;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-alerts': Alerts;
  }
}
