import { WebbitConfig } from '@webbitjs/webbit';
import { LitElement, css, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { map } from 'lit/directives/map.js';
import { globalStylesCss } from '../styles/styles';

export const commandsDashboardConfig: Partial<WebbitConfig> = {
  dashboard: {
    displayName: 'Scheduler',
  },
  properties: {
    cancel: { type: 'Array', defaultValue: [] },
    ids: { type: 'Array', defaultValue: [] },
    names: { type: 'Array', defaultValue: [] },
  },
};

@customElement('team1701-commands')
export class Commands extends LitElement {
  @property({ type: Array }) cancel: number[] = [];
  @property({ type: Array }) ids: number[] = [];
  @property({ type: Array }) names: string[] = [];

  render() {
    return html`<div class="flex flex-col gap-4 h-full">
      <div class="flex flex-col gap-4 text-white">
        <div>Commands</div>
        <ul class="list-disc text-white ml-5">
          ${map(this.names, (e) => html`<li>${e}</li>`)}
        </ul>
      </div>
    </div>`;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-commands': Commands;
  }
}
