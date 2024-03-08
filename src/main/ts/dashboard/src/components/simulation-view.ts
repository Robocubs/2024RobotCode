import { consume } from '@lit/context';
import { LitElement, TemplateResult, css, html } from 'lit';
import { customElement } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from '../network-tables/network-tables';
import { globalStylesCss } from '../styles/styles';

@customElement('team1701-simulation-view')
export class SimulationView extends LitElement {
  @consume({ context: nt4Context }) private nt!: NetworkTables;

  render(): TemplateResult {
    return html`<div class="flow w-full h-full">
      <team1701-leds .colors="${this.nt.$value('/AdvantageKit/RealOutputs/LED', [])}"></team1701-leds>
    </div>`;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-simulation-view': SimulationView;
  }
}
